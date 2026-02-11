Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing

# --- CONFIGURAÇÃO E DESCOBERTA ARKHE(N) ---
$SqlitePath = "C:\tools\sqlite3.exe"
$TempDb = Join-Path $env:TEMP "PlexSnapshot.db"

function Get-PlexDB {
    $RegPath = "HKCU:\Software\Plex, Inc.\Plex Media Server"
    if (Test-Path $RegPath) {
        $Custom = (Get-ItemProperty $RegPath -Name "LocalAppDataPath" -ErrorAction SilentlyContinue).LocalAppDataPath
        if ($Custom) { return Join-Path $Custom "Plex Media Server\Plug-in Support\Databases\com.plexapp.plugins.library.db" }
    }
    return "$env:LOCALAPPDATA\Plex Media Server\Plug-in Support\Databases\com.plexapp.plugins.library.db"
}

# --- INTERFACE WINFORMS (THEME: DARK ARKHE) ---
$Form = New-Object Windows.Forms.Form
$Form.Text = "Arkhe(n) - Plex Missing Media v2.1"
$Form.Size = "900,700"
$Form.BackColor = "#121212"
$Form.ForeColor = "#00ff00"
$Form.Font = New-Object Drawing.Font("Consolas", 10)

$TabControl = New-Object Windows.Forms.TabControl
$TabControl.Dock = "Fill"
$Form.Controls.Add($TabControl)

foreach ($Category in @("TV Shows", "Movies", "Anime")) {
    $Tab = New-Object Windows.Forms.TabPage
    $Tab.Text = $Category
    $Tab.BackColor = "#1e1e1e"

    $BtnScan = New-Object Windows.Forms.Button
    $BtnScan.Text = "ATIVAR ESCANEAMENTO ($Category)"
    $BtnScan.Location = "20,20"
    $BtnScan.Size = "250,45"
    $BtnScan.FlatStyle = "Flat"
    $BtnScan.BackColor = "#004400"
    $BtnScan.Add_Click({ Start-Scan -Type $Category })

    $Tab.Controls.Add($BtnScan)
    $TabControl.TabPages.Add($Tab)
}

$LogBox = New-Object Windows.Forms.RichTextBox
$LogBox.Dock = "Right"
$LogBox.Width = 350
$LogBox.ReadOnly = $true
$LogBox.BackColor = "#000000"
$LogBox.ForeColor = "#00ff00"
$Form.Controls.Add($LogBox)

function Write-Log($msg) { $LogBox.AppendText("[$((Get-Date).ToString('HH:mm:ss'))] $msg`n"); $LogBox.ScrollToCaret() }

# --- LÓGICA DE ESCANEAMENTO COGNITIVO ---
function Start-Scan([string]$Type) {
    Write-Log "🧬 Ativando Linfócito de Integridade para $Type..."
    $DB = Get-PlexDB
    if (-not (Test-Path $DB)) {
        Write-Log "❌ ERRO: Fonte da Verdade não localizada em $DB"
        return
    }

    Write-Log "🛡️ Isolando banco de dados em Câmara de Snapshot..."
    Copy-Item $DB $TempDb -Force

    # Query Sagrada Integrada (Exemplo para TV)
    $Query = "SELECT parent.title, md.parent_index, mp.file
              FROM metadata_items md
              JOIN metadata_items parent ON md.parent_id = parent.id
              JOIN media_items mi ON md.id = mi.metadata_item_id
              JOIN media_parts mp ON mi.id = mp.media_item_id
              WHERE md.metadata_type = 4;"

    try {
        Write-Log "📡 Interrogando memória do Plex..."
        # Nota: Necessita sqlite3.exe no PATH
        $Raw = & $SqlitePath -csv $TempDb $Query | ConvertFrom-Csv -Header "Title","Season","Path"

        $Missing = $Raw | Where-Object { -not (Test-Path $_.Path) }

        Write-Log "📊 Resultado: $($Missing.Count) vácuos detectados na malha de arquivos."

        if ($Missing.Count -gt 0) {
            $S_loss = ($Missing.Count / $Raw.Count) * 100
            Write-Log "⚠️ Severidade de Perda (S_loss): $($S_loss.ToString('F2'))%"
            if ($S_loss -gt 50) { Write-Log "🚨 ALERTA: Colapso de Volume detectado!" }
        }
    }
    catch {
        Write-Log "❌ Falha na interrogagem. Verifique sqlite3.exe."
    }

    Remove-Item $TempDb -Force
    Write-Log "🏛️ Scan concluído. Paz de Fase."
}

$Form.ShowDialog()
