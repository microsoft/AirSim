Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing

# --- ARKHE(N) OS: MÓDULO DE PRESERVAÇÃO v2.1 "VIGILANTE AUTÔNOMO" ---

$SettingsFile = Join-Path $PSScriptRoot "arkhe_config.json"
$LogPath = Join-Path $PSScriptRoot "arkhe_scan.log"
$SqlitePath = "sqlite3.exe"
$TempDb = Join-Path $env:TEMP "PlexSnapshot.db"

# --- 1. MEMÓRIA DE LONGO PRAZO (CONFIGURAÇÕES) ---
function Load-Settings {
    if (Test-Path $SettingsFile) {
        try {
            return Get-Content $SettingsFile -Raw | ConvertFrom-Json
        } catch {
            Write-Log "Falha ao ler config.json. Usando padrões." "WARN"
        }
    }
    $Default = @{
        SonarrUrl = "http://localhost:8989"
        SonarrKey = ""
        RadarrUrl = "http://localhost:7878"
        RadarrKey = ""
        RootFolderTV = "D:\Media\TV"
        RootFolderMovie = "D:\Media\Movies"
        AutoDetect = $true
    }
    $Default | ConvertTo-Json -Depth 3 | Set-Content $SettingsFile -Encoding UTF8
    return $Default
}

function Save-Settings {
    param($Settings)
    $Settings | ConvertTo-Json -Depth 3 | Set-Content $SettingsFile -Encoding UTF8
    Write-Log "Configurações persistidas em arkhe_config.json" "SUCCESS"
}

$Global:Settings = Load-Settings

# --- 2. MEMÓRIA EPISÓDICA (LOGGING) ---
function Write-Log {
    param([string]$Message, [ValidateSet("INFO", "ERROR", "WARN", "SUCCESS")]$Level = "INFO", $Component = "KERNEL")

    $Timestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss.fff"
    $Entry = "[$Timestamp] [$Level] [$Component] $Message"

    # Console/File
    $Color = switch ($Level) { "ERROR" { "Red" } "WARN" { "Yellow" } "SUCCESS" { "Green" } default { "Cyan" } }
    # Write-Host $Entry -ForegroundColor $Color # Disabled during GUI run for performance if needed
    Add-Content -Path $LogPath -Value $Entry -ErrorAction SilentlyContinue

    # GUI Update
    if ($LogBox) {
        $LogBox.Invoke([Action[string, string]]{
            param($m, $c)
            $LogBox.SelectionStart = $LogBox.TextLength
            $LogBox.SelectionLength = 0
            $LogBox.SelectionColor = [System.Drawing.ColorTranslator]::FromHtml($c)
            $LogBox.AppendText("$m`n")
            $LogBox.ScrollToCaret()
        }, $Entry, ([System.Drawing.ColorTranslator]::ToHtml([System.Drawing.Color]::$Color)))
    }
}

# --- 3. PULSO PERCEPTIVO (AUTO-DETECÇÃO) ---
function Get-PlexDB {
    $RegPath = "HKCU:\Software\Plex, Inc.\Plex Media Server"
    if (Test-Path $RegPath) {
        $Custom = (Get-ItemProperty $RegPath -Name "LocalAppDataPath" -ErrorAction SilentlyContinue).LocalAppDataPath
        if ($Custom) { return Join-Path $Custom "Plex Media Server\Plug-in Support\Databases\com.plexapp.plugins.library.db" }
    }
    return "$env:LOCALAPPDATA\Plex Media Server\Plug-in Support\Databases\com.plexapp.plugins.library.db"
}

function Detect-MissingDrives {
    Write-Log "Interrogando banco de dados para detecção de raízes órfãs..." "INFO" "DETECT"
    $DB = Get-PlexDB
    if (-not (Test-Path $DB)) { return $null }

    Copy-Item $DB $TempDb -Force
    $Query = "SELECT DISTINCT substr(file, 1, 3) FROM media_parts;"
    try {
        $Roots = & $SqlitePath -csv $TempDb $Query 2>$null | ForEach-Object { $_.Trim('"') }
        $Mounted = (Get-PSDrive -PSProvider FileSystem).Root
        $Missing = $Roots | Where-Object { $Mounted -notcontains $_ -and $_ -match "^[A-Z]:\\" }
        return $Missing
    } finally {
        Remove-Item $TempDb -Force -ErrorAction SilentlyContinue
    }
}

# --- 4. EMISSÁRIOS DE RESTAURAÇÃO (API INTEGRATION) ---
function Add-ToArr {
    param($Type, $Title, $Id, $Extra = $null)
    $Url = if ($Type -eq "TV") { $Settings.SonarrUrl } else { $Settings.RadarrUrl }
    $Key = if ($Type -eq "TV") { $Settings.SonarrKey } else { $Settings.RadarrKey }
    $Root = if ($Type -eq "TV") { $Settings.RootFolderTV } else { $Settings.RootFolderMovie }

    if (-not $Key -or -not $Url) { return }

    $Endpoint = if ($Type -eq "TV") { "$Url/api/v3/series" } else { "$Url/api/v3/movie" }
    $Payload = if ($Type -eq "TV") {
        @{ title = $Title; tvdbId = [int]$Id; monitored = $true; rootFolderPath = $Root; addOptions = @{ searchForMissingEpisodes = $true } }
    } else {
        @{ title = $Title; tmdbId = [int]$Id; year = [int]$Extra; monitored = $true; rootFolderPath = $Root; addOptions = @{ searchForMovie = $true } }
    }

    try {
        Invoke-RestMethod -Uri $Endpoint -Method Post -Body ($Payload | ConvertTo-Json -Depth 3) -ContentType "application/json" -Headers @{ "X-Api-Key" = $Key }
        Write-Log "Agente $Type: '$Title' enviado para restauração." "SUCCESS" "API"
    } catch {
        Write-Log "Falha ao comunicar com $Type para '$Title': $($_.Exception.Message)" "ERROR" "API"
    }
}

# --- 5. INTERFACE DO ARQUITETO ---
$Form = New-Object Windows.Forms.Form
$Form.Text = "Arkhe(n) OS - Plex Preservation v2.1"
$Form.Size = "900,700"
$Form.BackColor = "#1e1e1e"
$Form.ForeColor = "#ffffff"
$Form.StartPosition = "CenterScreen"

$TabControl = New-Object Windows.Forms.TabControl
$TabControl.Dock = "Fill"
$Form.Controls.Add($TabControl)

# Tab: Dashboard
$TabDash = New-Object Windows.Forms.TabPage; $TabDash.Text = "Dashboard"; $TabDash.BackColor = "#2d2d2d"
$BtnSmartFix = New-Object Windows.Forms.Button
$BtnSmartFix.Text = "SMART FIX (Auto-Detect & Sync)"
$BtnSmartFix.Location = "30,30"; $BtnScanTV.Size = "300,60" # Adjusted below
$BtnSmartFix.Size = "300,60"; $BtnSmartFix.FlatStyle = "Flat"; $BtnSmartFix.BackColor = "#005a9e"
$BtnSmartFix.Add_Click({ Run-SmartFix })
$TabDash.Controls.Add($BtnSmartFix)
$TabControl.TabPages.Add($TabDash)

# Tab: Settings
$TabSettings = New-Object Windows.Forms.TabPage; $TabSettings.Text = "Configurações"; $TabSettings.BackColor = "#2d2d2d"
$LabelStyle = { param($obj, $txt, $y) $obj.Text = $txt; $obj.Location = "20,$y"; $obj.Size = "200,20"; $obj.ForeColor = "#aaaaaa" }
$TextStyle = { param($obj, $val, $y, $x=20, $w=300) $obj.Text = $val; $obj.Location = "$x,$y"; $obj.Size = "$w,25"; $obj.BackColor = "#3e3e3e"; $obj.ForeColor = "#ffffff"; $obj.BorderStyle = "FixedSingle" }

$lblS = New-Object Windows.Forms.Label; & $LabelStyle $lblS "Sonarr URL / Key" 20
$txtSUrl = New-Object Windows.Forms.TextBox; & $TextStyle $txtSUrl $Settings.SonarrUrl 45
$txtSKey = New-Object Windows.Forms.TextBox; & $TextStyle $txtSKey $Settings.SonarrKey 45 330 250

$lblR = New-Object Windows.Forms.Label; & $LabelStyle $lblR "Radarr URL / Key" 80
$txtRUrl = New-Object Windows.Forms.TextBox; & $TextStyle $txtRUrl $Settings.RadarrUrl 105
$txtRKey = New-Object Windows.Forms.TextBox; & $TextStyle $txtRKey $Settings.RadarrKey 105 330 250

$lblPath = New-Object Windows.Forms.Label; & $LabelStyle $lblPath "Raiz TV / Filmes" 140
$txtPathT = New-Object Windows.Forms.TextBox; & $TextStyle $txtPathT $Settings.RootFolderTV 165
$txtPathM = New-Object Windows.Forms.TextBox; & $TextStyle $txtPathM $Settings.RootFolderMovie 165 330 250

$BtnSave = New-Object Windows.Forms.Button; $BtnSave.Text = "SALVAR ALTERAÇÕES"; $BtnSave.Location = "20,220"; $BtnSave.Size = "200,40"; $BtnSave.FlatStyle = "Flat"; $BtnSave.BackColor = "#2d7d46"
$BtnSave.Add_Click({
    $Settings.SonarrUrl = $txtSUrl.Text; $Settings.SonarrKey = $txtSKey.Text
    $Settings.RadarrUrl = $txtRUrl.Text; $Settings.RadarrKey = $txtRKey.Text
    $Settings.RootFolderTV = $txtPathT.Text; $Settings.RootFolderMovie = $txtPathM.Text
    Save-Settings -Settings $Settings
})
$TabSettings.Controls.AddRange(@($lblS, $txtSUrl, $txtSKey, $lblR, $txtRUrl, $txtRKey, $lblPath, $txtPathT, $txtPathM, $BtnSave))
$TabControl.TabPages.Add($TabSettings)

# Log Panel
$LogBox = New-Object Windows.Forms.RichTextBox
$LogBox.Dock = "Bottom"; $LogBox.Height = 350; $LogBox.BackColor = "#000000"; $LogBox.ForeColor = "#00ff00"; $LogBox.Font = New-Object Drawing.Font("Consolas", 9)
$Form.Controls.Add($LogBox)

# --- 6. ALGORITMO DE CAMPO (SMART FIX) ---
function Run-SmartFix {
    Write-Log "Iniciando Protocolo SMART FIX v2.1..." "INFO" "SYSTEM"

    $MissingDrives = Detect-MissingDrives
    if (-not $MissingDrives) {
        Write-Log "Nenhuma unidade órfã detectada. O campo está íntegro." "SUCCESS" "SYSTEM"
        return
    }

    Write-Log "UNIDADES AUSENTES DETECTADAS: $($MissingDrives -join ', ')" "WARN" "SYSTEM"

    foreach ($Type in @("TV", "Movie")) {
        Write-Log "Escaneando categoria: $Type..." "INFO" "SCAN"
        $DB = Get-PlexDB
        Copy-Item $DB $TempDb -Force

        $Query = if ($Type -eq "TV") {
            "WITH series_guids AS (SELECT id, title, CASE WHEN guid LIKE '%tvdb://%' THEN REPLACE(SUBSTR(guid, INSTR(guid, 'tvdb://') + 7), '?lang=pt', '') ELSE NULL END AS tvdb_id FROM metadata_items WHERE metadata_type = 2) SELECT sg.title, sg.tvdb_id, parts.file FROM metadata_items ep JOIN series_guids sg ON ep.parent_id = sg.id JOIN media_items mi ON mi.metadata_item_id = ep.id JOIN media_parts parts ON parts.media_item_id = mi.id WHERE ep.metadata_type = 4;"
        } else {
            "SELECT md.title, CASE WHEN md.guid LIKE '%tmdb://%' THEN REPLACE(SUBSTR(md.guid, INSTR(md.guid, 'tmdb://') + 7), '?lang=pt', '') ELSE NULL END AS tmdb_id, md.year, mp.file FROM metadata_items md JOIN media_items mi ON md.id = mi.metadata_item_id JOIN media_parts mp ON mi.id = mp.media_item_id WHERE md.metadata_type = 1;"
        }

        try {
            $Raw = & $SqlitePath -csv $TempDb $Query 2>$null | ConvertFrom-Csv -Header "Title","ID","Extra","Path"
            $Lost = $Raw | Where-Object { $itemPath = $_.Path; $MissingDrives | Where-Object { $itemPath.StartsWith($_) } }

            Write-Log "Encontrados $($Lost.Count) itens perdidos em unidades órfãs para $Type." "WARN" "SCAN"

            if ($Lost.Count -gt 0) {
                $Confirm = [System.Windows.Forms.MessageBox]::Show("Deseja enviar $($Lost.Count) itens de $Type para restauração automática via API?", "Confirmação Arkhe(n)", "YesNo", "Question")
                if ($Confirm -eq "Yes") {
                    foreach ($item in $Lost) {
                        Add-ToArr -Type $Type -Title $item.Title -Id $item.ID -Extra $item.Extra
                    }
                }
            }
        } catch {
            Write-Log "Erro catastrófico no scan: $($_.Exception.Message)" "ERROR" "SCAN"
        } finally {
            Remove-Item $TempDb -Force -ErrorAction SilentlyContinue
        }
    }
    Write-Log "Protocolo SMART FIX encerrado." "SUCCESS" "SYSTEM"
}

Write-Log "ARKHE(N) OS: MÓDULO DE PRESERVAÇÃO v2.1 ATIVO." "SUCCESS"
Write-Log "Φ = 1,000 | Aguardando diretrizes do Arquiteto."
$Form.ShowDialog()
