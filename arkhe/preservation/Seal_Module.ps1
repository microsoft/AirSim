# Arkhe(n) OS - Módulo de Preservação: Protocolo de Selagem
# Este script prepara o pacote de deploy Alfa para o Arquiteto.

$ProjectRoot = $PSScriptRoot
$ReleaseName = "Arkhe-MediaPreservation-v1.0.0"
$ReleaseDir = Join-Path $ProjectRoot $ReleaseName

Write-Host "[ARKHE] Iniciando Protocolo de Selagem para $ReleaseName..." -ForegroundColor Cyan

if (Test-Path $ReleaseDir) { Remove-Item $ReleaseDir -Recurse -Force }
New-Item -ItemType Directory -Path $ReleaseDir | Out-Null

# Criação de Subdiretórios
$BinDir = New-Item -ItemType Directory -Path (Join-Path $ReleaseDir "bin")
$DocsDir = New-Object System.IO.DirectoryInfo (Join-Path $ReleaseDir "docs")
$DocsDir.Create()
$SrcDir = New-Item -ItemType Directory -Path (Join-Path $ReleaseDir "src")

# Distribuição de Tecidos
Copy-Item (Join-Path $ProjectRoot "Axioma_Governanca.md") $DocsDir.FullName
Copy-Item (Join-Path $ProjectRoot "LOG_DA_CRIACAO.txt") $DocsDir.FullName
Copy-Item (Join-Path $ProjectRoot "PlexMissingMedia_GUI.ps1") $SrcDir
Copy-Item (Join-Path $ProjectRoot "SacredQuery.sql") $SrcDir
Copy-Item (Join-Path $ProjectRoot "Compile_Arkhe.bat") $ReleaseDir

# Cópias condicionais (se existirem no ambiente do Arquiteto)
if (Test-Path (Join-Path $ProjectRoot "PlexMissingMedia_GUI.exe")) {
    Copy-Item (Join-Path $ProjectRoot "PlexMissingMedia_GUI.exe") $BinDir
}
if (Test-Path (Join-Path $ProjectRoot "sqlite3.exe")) {
    Copy-Item (Join-Path $ProjectRoot "sqlite3.exe") $BinDir
}

# Geração de Hash de Integridade
$ManifestPath = Join-Path $DocsDir.FullName "MANIFEST.txt"
"ARKHE(N) OS - INTEGRITY MANIFEST`r`nGenerated: $((Get-Date).ToString('yyyy-MM-dd HH:mm:ss')) UTC`r`n" | Out-File $ManifestPath -Encoding UTF8
Get-ChildItem $ReleaseDir -Recurse -File | Get-FileHash | Out-String | Out-File $ManifestPath -Append -Encoding UTF8

# Compressão Final
$ZipFile = "$ReleaseDir.zip"
if (Test-Path $ZipFile) { Remove-Item $ZipFile -Force }
# Nota: Requer PowerShell 5.0+ para Compress-Archive
try {
    Compress-Archive -Path "$ReleaseDir\*" -DestinationPath $ZipFile -Force
    Write-Host "✅ Protocolo encerrado. Artefato gerado: $ZipFile" -ForegroundColor Green
} catch {
    Write-Host "⚠️ Erro ao gerar ZIP. Verifique se o Compress-Archive está disponível." -ForegroundColor Yellow
}

Write-Host "Φ = 1,000. Hibernando..." -ForegroundColor Yellow
