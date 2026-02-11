Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName System.Security

# --- ARKHE(N) OS v3.0: IDENTIDADE SOBERANA ---

# --- CONFIGURAÇÃO E AMBIENTE ---
$Script:ConfigPath = Join-Path $PSScriptRoot "arkhe_config.json"
$Script:LogPath = Join-Path $PSScriptRoot "arkhe_scan.log"
$Script:IdentityPath = Join-Path $PSScriptRoot "SIWA_IDENTITY.md"
$SqlitePath = "C:\tools\sqlite3.exe"
$TempDb = Join-Path $env:TEMP "PlexVigilante_$(Get-Random).db"

function Write-Log {
    param([string]$Message, [ValidateSet("INFO", "ERROR", "WARN", "SUCCESS")]$Level = "INFO")
    $Timestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
    $Entry = "[$Timestamp] [$Level] $Message"
    if ($LogBox) {
        $Color = switch($Level) { "ERROR" {'Red'} "WARN" {'Yellow'} "SUCCESS" {'LimeGreen'} default {'Cyan'} }
        $LogBox.Invoke([Action]{ $this.SelectionColor = [Drawing.Color]::$Color; $this.AppendText("$Entry`n"); $this.ScrollToCaret() })
    }
    Add-Content -Path $Script:LogPath -Value $Entry
}

# --- SIWA & KEYRING PROXY ---
function Sign-WithSIWA {
    param($Message, $Require2FA = $false, $Description = "")
    $Settings = Get-Settings
    $Timestamp = [DateTimeOffset]::Now.ToUnixTimeMilliseconds().ToString()
    $Body = @{ message = $Message; require2FA = $Require2FA; description = $Description } | ConvertTo-Json

    $HMAC_Msg = "POST:/sign-message:$Timestamp:$Body"
    $HMAC = New-Object Security.Cryptography.HMACSHA256
    $HMAC.Key = [Text.Encoding]::UTF8.GetBytes($Settings.ProxySecret)
    $Signature = [Convert]::ToHexString($HMAC.ComputeHash([Text.Encoding]::UTF8.GetBytes($HMAC_Msg))).ToLower()

    $Headers = @{ "X-Proxy-Signature" = $Signature; "X-Proxy-Timestamp" = $Timestamp }
    try {
        $Res = Invoke-RestMethod -Uri "$($Settings.ProxyURL)/sign-message" -Method Post -Body $Body -Headers $Headers -ContentType "application/json"
        return $Res.signature
    } catch {
        Write-Log "Falha na assinatura SIWA: $_" "ERROR"
        return $null
    }
}

# --- GUI ---
$Form = New-Object Windows.Forms.Form
$Form.Text = "Arkhe(n) OS - Identidade Soberana v3.0"
$Form.Size = "1000,850"
$Form.BackColor = "#050505"
$Form.ForeColor = "#00ff00"

$LogBox = New-Object Windows.Forms.RichTextBox
$LogBox.Dock = "Bottom"; $LogBox.Height = 300; $LogBox.BackColor = "#000000"; $LogBox.ForeColor = "#00ff00"
$Form.Controls.Add($LogBox)

$BtnSmartFix = New-Object Windows.Forms.Button
$BtnSmartFix.Text = "🧬 SMART FIX (SIWA PROTECTED)"
$BtnSmartFix.Size = "300,80"; $BtnSmartFix.Location = "50,100"; $BtnSmartFix.FlatStyle = "Flat"
$BtnSmartFix.Add_Click({ Start-SovereignFix })
$Form.Controls.Add($BtnSmartFix)

function Start-SovereignFix {
    Write-Log "Iniciando Protocolo de Identidade Soberana..."
    # Lógica de Scan e Auto-Detecção...
    # Se precisar restaurar:
    $Sig = Sign-WithSIWA -Message "RestoreRequest:$(Get-Random)" -Require2FA $true -Description "Restauração de 15 episódios de 'Family Guy'"
    if ($Sig) { Write-Log "Assinatura obtida: $Sig" "SUCCESS" }
}

$Form.ShowDialog()
