@echo off
echo [ARKHE(N)] Compilando modulo de preservacao...
powershell -Command "if (Get-Command ps2exe -ErrorAction SilentlyContinue) { ps2exe .\PlexMissingMedia_GUI.ps1 .\PlexMissingMedia_GUI.exe -icon .\plex_icon.ico -title 'Plex Missing Media' -noConsole } else { Write-Warning 'PS2EXE nao encontrado. O script permanecera em formato .ps1' }"
echo [OK] Processo de amalgama concluido.
pause
