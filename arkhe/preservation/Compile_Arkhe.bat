@echo off
setlocal
echo [ARKHE(N)] Compilando modulo de preservacao...

:: Verifica se o PS2EXE está disponível (via PowerShell)
powershell -Command "if (Get-Command ps2exe -ErrorAction SilentlyContinue) { exit 0 } else { exit 1 }"
if %ERRORLEVEL% NEQ 0 (
    echo [ERRO] PS2EXE não encontrado. Instale com: Install-Module -Name ps2exe
    pause
    exit /b 1
)

:: Compilação
powershell -Command "ps2exe .\PlexMissingMedia_GUI.ps1 .\PlexMissingMedia_GUI.exe -title 'Plex Missing Media' -noConsole"

if %ERRORLEVEL% EQU 0 (
    echo [OK] Executavel gerado: PlexMissingMedia_GUI.exe
) else (
    echo [FALHA] Erro na compilação.
)

pause
