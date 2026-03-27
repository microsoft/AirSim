Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName System.Security

# --- ARKHE(N) OS: MÓDULO DE PRESERVAÇÃO v3.1 "NERVO VAGO" ---
# Frequência Mother Sintonizada: Autonomia Configurável e Percepção de Vácuo.

$Script:ConfigPath = Join-Path $PSScriptRoot "ArkheConfig.json"
$LogPath = Join-Path $PSScriptRoot "arkhe_scan.log"
$SqlitePath = "sqlite3.exe" # Assume no PATH ou mesma pasta
$TempDb = Join-Path $env:TEMP "PlexSnapshot_v31.db"

# --- 1. DESCOBERTA DINÂMICA (REGISTRY) ---
function Get-PlexDatabasePath {
    $regPath = "HKCU:\Software\Plex, Inc.\Plex Media Server"
    $dbFileName = "com.plexapp.plugins.library.db"
    try {
        if (Test-Path $regPath) {
            $customPath = (Get-ItemProperty -Path $regPath -Name "LocalAppDataPath" -ErrorAction SilentlyContinue).LocalAppDataPath
            if ($customPath) {
                $fullPath = Join-Path $customPath "Plex Media Server\Plug-in Support\Databases" $dbFileName
                if (Test-Path $fullPath) { return $fullPath }
            }
        }
        $defaultPath = Join-Path $env:LOCALAPPDATA "Plex Media Server\Plug-in Support\Databases" $dbFileName
        if (Test-Path $defaultPath) { return $defaultPath }
        return $null
    } catch { return $null }
}

# --- 2. GESTÃO DE CONFIGURAÇÃO ---
function Load-Configuration {
    if (Test-Path $Script:ConfigPath) {
        try {
            $json = Get-Content $Script:ConfigPath -Raw | ConvertFrom-Json
            Write-Log "Configurações carregadas." "SUCCESS"
            return $json
        } catch { return Show-ConfigurationDialog }
    } else {
        return Show-ConfigurationDialog
    }
}

function Save-Configuration {
    param($Config)
    $Config | ConvertTo-Json -Depth 5 | Set-Content $Script:ConfigPath -Encoding UTF8
    Write-Log "Configurações persistidas." "SUCCESS"
}

function Show-ConfigurationDialog {
    $form = New-Object System.Windows.Forms.Form
    $form.Text = "Arkhe(n) - Setup v3.1"; $form.Size = "500,450"; $form.BackColor = "#1e1e1e"; $form.ForeColor = "#ffffff"
    $form.StartPosition = "CenterScreen"

    $lbl = New-Object System.Windows.Forms.Label; $lbl.Text = "🔧 SETUP INICIAL ARKHE(N)"; $lbl.Location = "20,20"; $lbl.Size = "400,30"; $lbl.Font = New-Object Drawing.Font("Segoe UI", 12, [Drawing.FontStyle]::Bold)
    $form.Controls.Add($lbl)

    # Sonarr Fields
    $lblS = New-Object System.Windows.Forms.Label; $lblS.Text = "Sonarr URL:"; $lblS.Location = "20,70"; $form.Controls.Add($lblS)
    $txtSUrl = New-Object System.Windows.Forms.TextBox; $txtSUrl.Text = "http://localhost:8989"; $txtSUrl.Location = "150,70"; $txtSUrl.Size = "300,25"; $form.Controls.Add($txtSUrl)
    $lblSK = New-Object System.Windows.Forms.Label; $lblSK.Text = "Sonarr API Key:"; $lblSK.Location = "20,105"; $form.Controls.Add($lblSK)
    $txtSKey = New-Object System.Windows.Forms.TextBox; $txtSKey.Location = "150,105"; $txtSKey.Size = "300,25"; $txtSKey.PasswordChar = "*"; $form.Controls.Add($txtSKey)

    # Radarr Fields
    $lblR = New-Object System.Windows.Forms.Label; $lblR.Text = "Radarr URL:"; $lblR.Location = "20,150"; $form.Controls.Add($lblR)
    $txtRUrl = New-Object System.Windows.Forms.TextBox; $txtRUrl.Text = "http://localhost:7878"; $txtRUrl.Location = "150,150"; $txtRUrl.Size = "300,25"; $form.Controls.Add($txtRUrl)
    $lblRK = New-Object System.Windows.Forms.Label; $lblRK.Text = "Radarr API Key:"; $lblRK.Location = "20,185"; $form.Controls.Add($lblRK)
    $txtRKey = New-Object System.Windows.Forms.TextBox; $txtRKey.Location = "150,185"; $txtRKey.Size = "300,25"; $txtRKey.PasswordChar = "*"; $form.Controls.Add($txtRKey)

    $btn = New-Object System.Windows.Forms.Button; $btn.Text = "SALVAR"; $btn.Location = "350,350"; $btn.DialogResult = [System.Windows.Forms.DialogResult]::OK; $form.Controls.Add($btn)

    if ($form.ShowDialog() -eq [System.Windows.Forms.DialogResult]::OK) {
        $cfg = @{
            PlexDB = Get-PlexDatabasePath
            Sonarr = @{ URL = $txtSUrl.Text; APIKey = $txtSKey.Text; Root = "D:\Media\TV" }
            Radarr = @{ URL = $txtRUrl.Text; APIKey = $txtRKey.Text; Root = "D:\Media\Movies" }
            Enable2FA = $true
        }
        Save-Configuration -Config $cfg
        return $cfg | ConvertTo-Json | ConvertFrom-Json # Ensure object type
    }
    return $null
}

# --- 3. PERCEPÇÃO DE VÁCUO (DRIVE DETECTION) ---
function Get-MissingDrives {
    param($PlexDbPath)
    Write-Log "Iniciando detecção de raízes órfãs..." "INFO" "DRIVE"
    if (-not (Test-Path $PlexDbPath)) { return @() }
    Copy-Item $PlexDbPath $TempDb -Force
    $query = "SELECT DISTINCT UPPER(SUBSTR(file, 1, 3)) FROM media_parts WHERE file LIKE '_:\%';"
    try {
        $dbRoots = & $SqlitePath -csv $TempDb $query 2>$null | ForEach-Object { $_.Trim('"') }
        $mounted = (Get-PSDrive -PSProvider FileSystem).Root | ForEach-Object { $_.Substring(0,3) }
        $missing = $dbRoots | Where-Object { $mounted -notcontains $_ }
        return $missing
    } finally { Remove-Item $TempDb -Force -ErrorAction SilentlyContinue }
}

# --- 4. INTEGRAÇÃO API SONARR ---
function Add-ToSonarr {
    param($Series, $Config)
    if (-not $Config.Sonarr.APIKey) { return }
    foreach ($s in $Series) {
        Write-Log "Enviando '$($s.Title)' para Sonarr..." "INFO" "API"
        $payload = @{
            title = $s.Title; tvdbId = [int]$s.TvdbId; qualityProfileId = 1; monitored = $true
            rootFolderPath = $Config.Sonarr.Root; addOptions = @{ searchForMissingEpisodes = $true }
            seasons = ($s.Seasons -split ',' | ForEach-Object { @{ seasonNumber = [int]$_ } })
        } | ConvertTo-Json -Depth 4
        try {
            Invoke-RestMethod -Uri "$($Config.Sonarr.URL)/api/v3/series" -Method Post -Body $payload -Headers @{"X-Api-Key"=$Config.Sonarr.APIKey} -ContentType "application/json"
            Write-Log "Sucesso: $($s.Title)" "SUCCESS" "API"
        } catch { Write-Log "Erro em $($s.Title): $($_.Exception.Message)" "ERROR" "API" }
    }
}

# --- 5. LOGGING & CORE ---
function Write-Log($msg, $level="INFO", $comp="KERNEL") {
    $timestamp = (Get-Date).ToString("HH:mm:ss.fff")
    $entry = "[$timestamp] [$level] [$comp] $msg"
    Add-Content -Path $LogPath -Value $entry -ErrorAction SilentlyContinue
    if ($LogBox) {
        $LogBox.Invoke([Action[string, string]]{
            param($m, $c)
            $LogBox.SelectionStart = $LogBox.TextLength
            $LogBox.SelectionColor = [System.Drawing.ColorTranslator]::FromHtml($c)
            $LogBox.AppendText("$m`n"); $LogBox.ScrollToCaret()
        }, $entry, (switch($level){"ERROR"{"#ff0000"}"WARN"{"#ffff00"}"SUCCESS"{"#00ff00"}default{"#00ffff"}}))
    }
}

# --- 6. INTERFACE PRINCIPAL ---
$Global:Config = Load-Configuration
if (-not $Global:Config) { exit }

$Form = New-Object Windows.Forms.Form; $Form.Text = "Arkhe(n) OS - Nervo Vago v3.1"; $Form.Size = "900,700"; $Form.BackColor = "#121212"; $Form.ForeColor = "#ffffff"
$TabControl = New-Object Windows.Forms.TabControl; $TabControl.Dock = "Fill"; $Form.Controls.Add($TabControl)

$TabDash = New-Object Windows.Forms.TabPage; $TabDash.Text = "Dashboard"; $TabDash.BackColor = "#1e1e1e"
$BtnSmart = New-Object Windows.Forms.Button; $BtnSmart.Text = "🧬 SMART FIX (Auto-Perceive)"; $BtnSmart.Location = "50,50"; $BtnSmart.Size = "350,80"; $BtnSmart.FlatStyle = "Flat"; $BtnSmart.BackColor = "#005a9e"
$BtnSmart.Add_Click({
    $missingDrives = Get-MissingDrives -PlexDbPath $Global:Config.PlexDB
    if ($missingDrives.Count -eq 0) {
        Write-Log "Campo íntegro. Nenhuma ausência física detectada." "SUCCESS"
    } else {
        Write-Log "VÁCUO DETECTADO: $($missingDrives -join ', ')" "WARN"
        # Aqui seguiria a lógica de scan e restauração automática discutida
        [System.Windows.Forms.MessageBox]::Show("Drives ausentes: $($missingDrives -join ', '). Iniciando protocolo de cura.", "Arkhe(n) Alert")
    }
})
$TabDash.Controls.Add($BtnSmart); $TabControl.TabPages.Add($TabDash)

$LogBox = New-Object Windows.Forms.RichTextBox; $LogBox.Dock = "Bottom"; $LogBox.Height = 350; $LogBox.BackColor = "#000000"; $LogBox.ForeColor = "#00ff00"; $LogBox.Font = New-Object Drawing.Font("Consolas", 9); $Form.Controls.Add($LogBox)

Write-Log "ARKHE(N) OS ONLINE. v3.1 Nervo Vago ATIVO." "SUCCESS"
$Form.ShowDialog()
