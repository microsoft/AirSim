Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName System.Security

# --- ARKHE(N) OS: MÓDULO DE PRESERVAÇÃO v3.0 "IDENTIDADE SOBERANA" ---
$SettingsFile = Join-Path $PSScriptRoot "arkhe_config.json"
$IdentityFile = Join-Path $PSScriptRoot "SIWA_IDENTITY.md"
$LogPath = Join-Path $PSScriptRoot "arkhe_scan.log"
$SqlitePath = "sqlite3.exe"
$TempDb = Join-Path $env:TEMP "PlexSnapshot.db"

# --- 1. GESTÃO DE SETTINGS (DPAPI) ---
function Load-Settings {
    if (Test-Path $SettingsFile) {
        try {
            $bytes = [System.IO.File]::ReadAllBytes($SettingsFile)
            $decrypted = [System.Security.Cryptography.ProtectedData]::Unprotect($bytes, $null, [System.Security.Cryptography.DataProtectionScope]::CurrentUser)
            return [System.Text.Encoding]::UTF8.GetString($decrypted) | ConvertFrom-Json
        } catch { Write-Log "Erro ao ler config. Recriando." "WARN" }
    }
    $Default = @{
        SonarrUrl = "http://localhost:8989"; SonarrKey = ""; RadarrUrl = "http://localhost:7878"; RadarrKey = ""
        RootFolderTV = "D:\Media\TV"; RootFolderMovie = "D:\Media\Movies"
        SiwaProxyUrl = "http://localhost:3000"; SiwaProxySecret = ""; Enable2FA = $true; GatewayUrl = "http://localhost:4000"
    }
    Save-Settings -Settings $Default; return $Default
}

function Save-Settings {
    param($Settings)
    $bytes = [System.Text.Encoding]::UTF8.GetBytes(($Settings | ConvertTo-Json -Depth 3))
    $encrypted = [System.Security.Cryptography.ProtectedData]::Protect($bytes, $null, [System.Security.Cryptography.DataProtectionScope]::CurrentUser)
    [System.IO.File]::WriteAllBytes($SettingsFile, $encrypted)
    Write-Log "Configurações cifradas e salvas." "SUCCESS"
}

# --- 2. IDENTIDADE & ASSINATURA ---
function Read-SiwaIdentity {
    if (-not (Test-Path $IdentityFile)) { return $null }
    $content = Get-Content $IdentityFile -Raw
    return @{
        Address = ([regex]::Match($content, "Address:\*\* `?(0x[a-fA-F0-9]+)`?")).Groups[1].Value
        AgentId = ([regex]::Match($content, "Agent ID:\*\* `?(\d+)`?")).Groups[1].Value
    }
}

function Get-ProxyHMAC {
    param($Method, $Path, $Timestamp, $Body, $Secret)
    $message = "$($Method):$($Path):$($Timestamp):$($Body)"
    $hmac = New-Object System.Security.Cryptography.HMACSHA256
    $hmac.Key = [System.Text.Encoding]::UTF8.GetBytes($Secret)
    return [Convert]::ToBase64String($hmac.ComputeHash([System.Text.Encoding]::UTF8.GetBytes($message)))
}

function Invoke-2FAApproval {
    param($OpId, $Desc)
    Write-Log "Solicitando aprovação 2FA via Telegram (Op: $OpId)..." "WARN" "SIWA"
    $timestamp = [DateTimeOffset]::Now.ToUnixTimeMilliseconds()
    $body = @{ operationId = $OpId; description = $Desc } | ConvertTo-Json
    $hmac = Get-ProxyHMAC -Method "POST" -Path "/request-approval" -Timestamp $timestamp -Body $body -Secret $Settings.SiwaProxySecret

    try {
        Invoke-RestMethod -Uri "$($Settings.GatewayUrl)/request-approval" -Method Post -Body $body -Headers @{ "x-proxy-signature" = $hmac; "x-proxy-timestamp" = $timestamp } -ContentType "application/json"
        $start = Get-Date
        while (((Get-Date) - $start).TotalMinutes -lt 5) {
            $status = Invoke-RestMethod -Uri "$($Settings.GatewayUrl)/approval-status/$OpId" -Method Get
            if ($status.approved -eq $true) { Write-Log "✅ Operação Aprovada via Telegram." "SUCCESS" "SIWA"; return $true }
            if ($status.approved -eq $false) { Write-Log "❌ Operação Rejeitada via Telegram." "ERROR" "SIWA"; return $false }
            Start-Sleep -Seconds 2
        }
    } catch { Write-Log "Erro 2FA: $($_.Exception.Message)" "ERROR" }
    return $false
}

# --- 3. LOGGING & GUI ---
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

$Global:Settings = Load-Settings; $Global:Identity = Read-SiwaIdentity

# --- INTERFACE ---
$Form = New-Object Windows.Forms.Form; $Form.Text = "Arkhe(n) OS - v3.0 [SIWA]"; $Form.Size = "900,700"; $Form.BackColor = "#1e1e1e"; $Form.ForeColor = "#ffffff"
$TabControl = New-Object Windows.Forms.TabControl; $TabControl.Dock = "Fill"; $Form.Controls.Add($TabControl)

# Tab: Dashboard
$TabDash = New-Object Windows.Forms.TabPage; $TabDash.Text = "Dashboard"; $TabDash.BackColor = "#2d2d2d"
$BtnSmartFix = New-Object Windows.Forms.Button; $BtnSmartFix.Text = "🧬 SMART FIX (SIWA SECURE)"; $BtnSmartFix.Location = "50,50"; $BtnSmartFix.Size = "400,100"; $BtnSmartFix.FlatStyle = "Flat"; $BtnSmartFix.BackColor = "#005a9e"
$BtnSmartFix.Add_Click({
    Write-Log "Iniciando Smart Fix..."
    $OpId = [guid]::NewGuid().ToString()
    if ($Settings.Enable2FA) {
        if (-not (Invoke-2FAApproval -OpId $OpId -Desc "Smart Fix: Scan & Restoration")) {
            Write-Log "Operação cancelada ou tempo esgotado." "ERROR" "SIWA"
            return
        }
    }
    Write-Log "Smart Fix em execução com Identidade Soberana: $($Identity.Address)" "SUCCESS"
    # Lógica de scan aqui...
})
$TabDash.Controls.Add($BtnSmartFix); $TabControl.TabPages.Add($TabDash)

$LogBox = New-Object Windows.Forms.RichTextBox; $LogBox.Dock = "Bottom"; $LogBox.Height = 350; $LogBox.BackColor = "#000000"; $LogBox.ForeColor = "#00ff00"; $LogBox.Font = New-Object Drawing.Font("Consolas", 9); $Form.Controls.Add($LogBox)

Write-Log "ARKHE(N) OS ONLINE. v3.0 Identidade Soberana." "SUCCESS"
$Form.ShowDialog()
