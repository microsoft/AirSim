Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName System.Security

# --- ARKHE(N) OS: MÓDULO DE PRESERVAÇÃO v3.1 "NERVO VAGO" ---

$SettingsFile = Join-Path $PSScriptRoot "ArkheConfig.json"
$IdentityFile = Join-Path $PSScriptRoot "SIWA_IDENTITY.md"
$LogPath = Join-Path $PSScriptRoot "arkhe_scan.log"
$SqlitePath = "sqlite3.exe"
$TempDb = Join-Path $env:TEMP "PlexSnapshot.db"

# --- 1. DESCOBERTA DINÂMICA DO BANCO (REGISTRY) ---
function Get-PlexDBPath {
    $RegPath = "HKCU:\Software\Plex, Inc.\Plex Media Server"
    $ValueName = "LocalAppDataPath"
    $DefaultPath = "$env:LOCALAPPDATA\Plex Media Server\Plug-in Support\Databases\com.plexapp.plugins.library.db"

    if (Test-Path $RegPath) {
        $CustomPath = (Get-ItemProperty $RegPath -Name $ValueName -ErrorAction SilentlyContinue).$ValueName
        if ($CustomPath) {
            $FinalPath = Join-Path $CustomPath "Plex Media Server\Plug-in Support\Databases\com.plexapp.plugins.library.db"
            if (Test-Path $FinalPath) { return $FinalPath }
        }
    }
    return $DefaultPath
}

# --- 2. GESTÃO DE SETTINGS ---
function Load-Settings {
    if (Test-Path $SettingsFile) {
        try {
            $json = Get-Content $SettingsFile -Raw | ConvertFrom-Json
            # Em v3.1, se o JSON tiver keys sensíveis, podemos criptografar opcionalmente
            return $json
        } catch { Write-Log "Erro ao ler ArkheConfig.json." "WARN" }
    }
    $Default = @{
        PlexDBPath = Get-PlexDBPath
        Sonarr = @{ Url = "http://localhost:8989"; Key = ""; RootPath = "D:\Media\TV" }
        Radarr = @{ Url = "http://localhost:7878"; Key = ""; RootPath = "D:\Media\Movies" }
        Siwa = @{ ProxyUrl = "http://localhost:3000"; ProxySecret = ""; Enable2FA = $true; GatewayUrl = "http://localhost:4000" }
    }
    Save-Settings -Settings $Default
    return $Default
}

function Save-Settings {
    param($Settings)
    $Settings | ConvertTo-Json -Depth 5 | Set-Content $SettingsFile -Encoding UTF8
    Write-Log "Configurações salvas em ArkheConfig.json." "SUCCESS"
}

# --- 3. DETECÇÃO DE UNIDADES ÓRFÃS ---
function Get-MissingDrivesFromDB {
    $DB = $Settings.PlexDBPath
    if (-not (Test-Path $DB)) { Write-Log "Banco de dados não encontrado!" "ERROR"; return @() }

    Copy-Item $DB $TempDb -Force
    $Query = "SELECT DISTINCT substr(file, 1, 3) FROM media_parts WHERE file LIKE '_:\%';"
    try {
        $DrivesInDB = & $SqlitePath -csv $TempDb $Query 2>$null | ForEach-Object { $_.Trim('"') }
        $Mounted = (Get-PSDrive -PSProvider FileSystem).Root
        $Missing = $DrivesInDB | Where-Object { $Mounted -notcontains $_ }

        if ($Missing.Count -gt 1) {
            # Simulação de prompt (na GUI real usaríamos um ComboBox)
            Write-Log "Múltiplos drives ausentes detectados: $($Missing -join ', ')" "WARN"
        }
        return $Missing
    } finally {
        Remove-Item $TempDb -Force -ErrorAction SilentlyContinue
    }
}

# --- 4. INTEGRAÇÃO APIs (SONARR/RADARR) ---
function Add-ToSonarr {
    param($Title, $TvdbId, $Seasons, $RootPath, $ProfileId = 1)
    if (-not $Settings.Sonarr.Key) { return }

    $SeasonList = $Seasons -split ',' | ForEach-Object { @{ seasonNumber = [int]$_; monitored = $true } }
    $Payload = @{
        title = $Title
        tvdbId = [int]$TvdbId
        seasons = $SeasonList
        rootFolderPath = $RootPath
        qualityProfileId = $ProfileId
        monitored = $true
        addOptions = @{ searchForMissingEpisodes = $true }
    } | ConvertTo-Json -Depth 5

    try {
        Invoke-RestMethod -Uri "$($Settings.Sonarr.Url)/api/v3/series" -Method Post -Body $Payload -Headers @{ "X-Api-Key" = $Settings.Sonarr.Key } -ContentType "application/json"
        Write-Log "Sonarr: '$Title' adicionada com sucesso." "SUCCESS" "API"
    } catch {
        Write-Log "Erro Sonarr para '$Title': $($_.Exception.Message)" "ERROR" "API"
    }
}

# --- 5. LOGGING & SIWA HELPERS ---
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

function Invoke-2FAApproval {
    param($OpId, $Desc)
    Write-Log "Solicitando 2FA para o Arquiteto..." "WARN" "SIWA"
    # Simulação de envio HMAC para o Gateway
    # [Lógica v3.0 completa omitida para brevidade nesta função de controle]
    return $true # Simulando aprovação
}

# --- 6. INTERFACE ---
$Settings = Load-Settings
$Form = New-Object Windows.Forms.Form; $Form.Text = "Arkhe(n) OS - Nervo Vago v3.1"; $Form.Size = "950,750"; $Form.BackColor = "#1e1e1e"; $Form.ForeColor = "#ffffff"
$TabControl = New-Object Windows.Forms.TabControl; $TabControl.Dock = "Fill"; $Form.Controls.Add($TabControl)

$TabDash = New-Object Windows.Forms.TabPage; $TabDash.Text = "Dashboard"; $TabDash.BackColor = "#2d2d2d"
$BtnSmart = New-Object Windows.Forms.Button; $BtnSmart.Text = "🧬 SMART FIX (Auto-Detect)"; $BtnSmart.Location = "30,30"; $BtnSmart.Size = "300,60"; $BtnSmart.FlatStyle = "Flat"; $BtnSmart.BackColor = "#005a9e"
$BtnSmart.Add_Click({
    $missing = Get-MissingDrivesFromDB
    if ($missing.Count -eq 0) { Write-Log "Nenhum vácuo detectado." "SUCCESS" }
    else { Write-Log "Drives ausentes: $($missing -join ', ')" "WARN" }
})
$TabDash.Controls.Add($BtnSmart); $TabControl.TabPages.Add($TabDash)

$LogBox = New-Object Windows.Forms.RichTextBox; $LogBox.Dock = "Bottom"; $LogBox.Height = 350; $LogBox.BackColor = "#000000"; $LogBox.ForeColor = "#00ff00"; $Form.Controls.Add($LogBox)

Write-Log "ARKHE(N) OS: NERVO VAGO v3.1 ATIVO." "SUCCESS"
$Form.ShowDialog()
