Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName System.Security

# --- ARKHE(N) OS v3.0: IDENTIDADE SOBERANA ---
# "A memória não é o que guardamos, mas o que somos capazes de restaurar."

# --- CONFIGURAÇÃO E AMBIENTE ---
$Script:ConfigPath = Join-Path $PSScriptRoot "arkhe_config.json"
$Script:LogPath = Join-Path $PSScriptRoot "arkhe_scan.log"
$Script:IdentityPath = Join-Path $PSScriptRoot "SIWA_IDENTITY.md"
$SqlitePath = "sqlite3.exe" # Assume it's in PATH or current directory
$TempDb = Join-Path $env:TEMP "PlexVigilante_$(Get-Random).db"

# --- MÓDULO DE SEGURANÇA (DPAPI) ---
function Protect-Config {
    param($ConfigObject)
    $json = $ConfigObject | ConvertTo-Json -Depth 5
    $bytes = [System.Text.Encoding]::UTF8.GetBytes($json)
    $encrypted = [System.Security.Cryptography.ProtectedData]::Protect($bytes, $null, [System.Security.Cryptography.DataProtectionScope]::CurrentUser)
    [System.IO.File]::WriteAllBytes($Script:ConfigPath, $encrypted)
    Write-Log "Configurações cifradas com DPAPI – selo de carne aplicado." "SUCCESS"
}

function Unprotect-Config {
    if (-not (Test-Path $Script:ConfigPath)) { return $null }
    try {
        $encrypted = [System.IO.File]::ReadAllBytes($Script:ConfigPath)
        $bytes = [System.Security.Cryptography.ProtectedData]::Unprotect($encrypted, $null, [System.Security.Cryptography.DataProtectionScope]::CurrentUser)
        $json = [System.Text.Encoding]::UTF8.GetString($bytes)
        return ($json | ConvertFrom-Json)
    } catch {
        Write-Log "Falha ao descriptografar configurações. Usuário incorreto ou arquivo corrompido." "ERROR"
        return $null
    }
}

# --- MÓDULO DE LOGGING ---
function Write-Log {
    param(
        [string]$Message,
        [ValidateSet("INFO", "ERROR", "WARN", "SUCCESS")]$Level = "INFO",
        [string]$Component = "VIGILANTE"
    )
    $Timestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss.fff"
    $Entry = "[$Timestamp] [$Level] [$Component] $Message"

    if ($LogBox) {
        $Color = switch($Level) {
            "ERROR"   { [System.Drawing.Color]::Red }
            "WARN"    { [System.Drawing.Color]::Yellow }
            "SUCCESS" { [System.Drawing.Color]::LimeGreen }
            default   { [System.Drawing.Color]::Cyan }
        }
        $LogBox.Invoke([Action]{
            $this.SelectionStart = $this.TextLength
            $this.SelectionLength = 0
            $this.SelectionColor = $Color
            $this.AppendText("$Entry`n")
            $this.SelectionColor = $this.ForeColor
            $this.ScrollToCaret()
        })
    }

    Add-Content -Path $Script:LogPath -Value $Entry
}

# --- GESTÃO DE CONFIGURAÇÃO ---
function Get-Settings {
    $Settings = Unprotect-Config
    if ($null -eq $Settings) {
        Write-Log "Iniciando nova matriz de configurações." "WARN"
        $Settings = [PSCustomObject]@{
            PlexDbPath = "$env:LOCALAPPDATA\Plex Media Server\Plug-in Support\Databases\com.plexapp.plugins.library.db"
            Sonarr = @{ URL = "http://localhost:8989"; APIKey = ""; DefaultPath = "D:\Media\TV"; Active = $false }
            Radarr = @{ URL = "http://localhost:7878"; APIKey = ""; DefaultPath = "D:\Media\Movies"; Active = $false }
            ProxyURL = "http://localhost:3000"
            ProxySecret = ""
            AutoDetectDrive = $true
            ExportPath = Join-Path $env:USERPROFILE "Desktop\Arkhe_Recovery"
        }
        Protect-Config -ConfigObject $Settings
    }
    return $Settings
}

# --- SIWA & KEYRING PROXY ---
function Sign-WithSIWA {
    param($Message, $Require2FA = $false, $Description = "")
    $Settings = Get-Settings
    if (-not $Settings.ProxySecret) {
        Write-Log "Proxy Secret não configurado. Assinatura SIWA impossibilitada." "ERROR"
        return $null
    }

    $Timestamp = [DateTimeOffset]::Now.ToUnixTimeMilliseconds().ToString()
    $Body = @{
        message = $Message
        require2FA = $Require2FA
        description = $Description
        metadata = @{ severity = "critical" }
    } | ConvertTo-Json

    $HMAC_Msg = "POST:/sign-message:$Timestamp:$Body"
    $HMAC = New-Object Security.Cryptography.HMACSHA256
    $HMAC.Key = [Text.Encoding]::UTF8.GetBytes($Settings.ProxySecret)
    $Signature = [Convert]::ToHexString($HMAC.ComputeHash([Text.Encoding]::UTF8.GetBytes($HMAC_Msg))).ToLower()

    $Headers = @{
        "X-Proxy-Signature" = $Signature
        "X-Proxy-Timestamp" = $Timestamp
    }
    try {
        $Res = Invoke-RestMethod -Uri "$($Settings.ProxyURL)/sign-message" -Method Post -Body $Body -Headers $Headers -ContentType "application/json"
        return $Res.signature
    } catch {
        Write-Log "Falha na assinatura SIWA: $_" "ERROR"
        return $null
    }
}

# --- MOTOR DE DESCOBERTA ESPACIAL ---
function Get-PlexDB {
    param($Settings)
    if (Test-Path $Settings.PlexDbPath) { return $Settings.PlexDbPath }

    $RegPath = "HKCU:\Software\Plex, Inc.\Plex Media Server"
    if (Test-Path $RegPath) {
        $Custom = (Get-ItemProperty $RegPath -Name "LocalAppDataPath" -ErrorAction SilentlyContinue).LocalAppDataPath
        if ($Custom) {
            $Path = Join-Path $Custom "Plex Media Server\Plug-in Support\Databases\com.plexapp.plugins.library.db"
            if (Test-Path $Path) { return $Path }
        }
    }
    return $Settings.PlexDbPath
}

function Get-MissingDrives {
    param($TempDbPath)
    Write-Log "Interrogando HSI por vácuos de montagem..."
    $Query = "SELECT DISTINCT SUBSTR(file, 1, 3) FROM media_parts;"
    try {
        $DbRoots = & $SqlitePath -csv $TempDbPath $Query | ForEach-Object { $_.Trim('"') }
        $MountedDrives = (Get-PSDrive -PSProvider FileSystem | Select-Object -ExpandProperty Root)
        $Missing = $DbRoots | Where-Object { $MountedDrives -notcontains $_ -and $_ -match "^[A-Z]:\\" }
        if ($Missing) {
            Write-Log "Vácuo detectado nas unidades: $($Missing -join ', ')" "WARN"
            return $Missing
        }
    } catch {
        Write-Log "Falha na análise de raízes: $_" "ERROR"
    }
    return $null
}

# --- INTERFACE WINFORMS ---
$Form = New-Object Windows.Forms.Form
$Form.Text = "Arkhe(n) OS - Identidade Soberana v3.0"
$Form.Size = "1000,850"
$Form.BackColor = "#050505"
$Form.ForeColor = "#00ff00"
$Form.Font = New-Object Drawing.Font("Consolas", 10)

$LogBox = New-Object Windows.Forms.RichTextBox
$LogBox.Dock = "Bottom"
$LogBox.Height = 350
$LogBox.BackColor = "#000000"
$LogBox.ForeColor = "#00ff00"
$LogBox.ReadOnly = $true
$LogBox.BorderStyle = "None"
$Form.Controls.Add($LogBox)

$BtnSmartFix = New-Object Windows.Forms.Button
$BtnSmartFix.Text = "🧬 SMART FIX (SIWA PROTECTED)"
$BtnSmartFix.Size = "300,80"
$BtnSmartFix.Location = "50,80"
$BtnSmartFix.FlatStyle = "Flat"
$BtnSmartFix.BackColor = "#003300"
$BtnSmartFix.Add_Click({ Start-SovereignFix })
$Form.Controls.Add($BtnSmartFix)

$BtnSettings = New-Object Windows.Forms.Button
$BtnSettings.Text = "⚙️ SETTINGS"
$BtnSettings.Size = "150,80"
$BtnSettings.Location = "370,80"
$BtnSettings.FlatStyle = "Flat"
$BtnSettings.Add_Click({ [System.Diagnostics.Process]::Start("notepad.exe", $Script:ConfigPath) })
$Form.Controls.Add($BtnSettings)

# --- LÓGICA VIGILANTE ---
function Start-SovereignFix {
    $Settings = Get-Settings
    Write-Log "Iniciando Protocolo de Identidade Soberana..."

    $DbPath = Get-PlexDB -Settings $Settings
    if (-not (Test-Path $DbPath)) { Write-Log "ERRO: Fonte da Verdade ausente em $DbPath" "ERROR"; return }

    Write-Log "Criando Snapshot de Memória..."
    Copy-Item $DbPath $TempDb -Force

    try {
        $MissingDrives = Get-MissingDrives -TempDbPath $TempDb

        # Exemplo de loop de assinatura para operação crítica
        $Description = "Restaurar integridade da biblioteca no drive $($MissingDrives[0])"
        $Sig = Sign-WithSIWA -Message "RestoreRequest:$(Get-Random)" -Require2FA $true -Description $Description

        if ($Sig) {
            Write-Log "Assinatura SIWA obtida. Vontade autorizada." "SUCCESS"
            # Lógica de restauração prosseguiria aqui...
        }
    } finally {
        if (Test-Path $TempDb) { Remove-Item $TempDb -Force }
        Write-Log "Protocolo finalizado. Φ = 1.000" "SUCCESS"
    }
}

$Form.ShowDialog()
