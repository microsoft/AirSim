<#
.SYNOPSIS
    PlexIntegrity: Módulo de Integridade e Reaquisição do Arkhe(n) OS.
    Combate o "vazio informacional" detectando mídia ausente e preparando a restauração.
#>

function Get-PlexDatabasePath {
    $RegistryPath = "HKCU:\Software\Plex, Inc.\Plex Media Server"
    $ValueName = "LocalAppDataPath"
    $dbFileName = "com.plexapp.plugins.library.db"
    $DefaultPath = "$env:LOCALAPPDATA\Plex Media Server\Plug-in Support\Databases\$dbFileName"

    if (Test-Path $RegistryPath) {
        $CustomPath = Get-ItemProperty -Path $RegistryPath -Name $ValueName -ErrorAction SilentlyContinue
        if ($CustomPath -and $CustomPath.$ValueName) {
            $FinalPath = Join-Path $CustomPath.$ValueName "Plex Media Server\Plug-in Support\Databases\$dbFileName"
            if (Test-Path $FinalPath) { return $FinalPath }
        }
    }
    return $DefaultPath
}

function Invoke-PlexScan {
    param(
        [string]$PlexDbPath,
        [string]$MissingDriveRoot,
        [string]$OutputCsvPath,
        [string]$SqlitePath = "sqlite3.exe"
    )

    Write-Host "🔍 Iniciando Scan Arkhe(n) no banco: $PlexDbPath" -ForegroundColor Cyan

    # 1. Câmara de Isolamento (Cópia temporária)
    $tempDb = Join-Path $env:TEMP "arkhe_plex_scan_$(Get-Random).db"
    Copy-Item $PlexDbPath $tempDb -Force

    try {
        # 2. A Query Sagrada (TV Shows)
        $query = @"
        WITH series_guids AS (
            SELECT id, title,
            CASE
                WHEN guid LIKE '%thetvdb://%' THEN REPLACE(SUBSTR(guid, INSTR(guid, 'thetvdb://') + 10), '?lang=pt', '')
                WHEN guid LIKE '%tvdb://%' THEN REPLACE(SUBSTR(guid, INSTR(guid, 'tvdb://') + 7), '?lang=pt', '')
                ELSE NULL
            END AS tvdb_id
            FROM metadata_items WHERE metadata_type = 2 AND deleted_at IS NULL AND guid IS NOT NULL
        )
        SELECT sg.title, sg.tvdb_id, ep.parent_index, mp.file_path
        FROM metadata_items AS ep
        JOIN series_guids AS sg ON ep.parent_id = sg.id
        JOIN media_items AS mi ON ep.id = mi.metadata_item_id
        JOIN media_parts AS mp ON mi.id = mp.media_item_id
        WHERE ep.metadata_type = 4 AND ep.deleted_at IS NULL;
"@

        Write-Host "📡 Executando Query Sagrada..." -ForegroundColor Gray
        $rawCsv = & $SqlitePath -csv $tempDb $query 2>$null

        if (-not $rawCsv) {
            Write-Error "Falha ao extrair dados do banco ou banco vazio."
            return
        }

        # 3. Processamento e Cálculo de Severidade (S_loss)
        Write-Host "📊 Calculando Índice de Colapso de Volume..." -ForegroundColor Gray
        $data = $rawCsv | ConvertFrom-Csv -Header 'Title','TvdbId','Season','FilePath'

        $results = $data | Where-Object { $_.FilePath -like "${MissingDriveRoot}*" } |
            Group-Object Title, TvdbId | ForEach-Object {
                $totalInSeries = $_.Count
                $missingItems = $_.Group | Where-Object { -not (Test-Path $_.FilePath) }
                $missingCount = ($missingItems | Measure-Object).Count

                $severity = if ($totalInSeries -gt 0) { [math]::Round(($missingCount / $totalInSeries) * 100, 2) } else { 0 }

                [PSCustomObject]@{
                    Title    = $_.Values[0]
                    TvdbId   = $_.Values[1]
                    Seasons  = ($missingItems.Season | Select-Object -Unique | Sort-Object) -join ','
                    Missing  = $missingCount
                    Total    = $totalInSeries
                    Severity = "$severity%"
                }
            } | Where-Object { $_.Missing -gt 0 }

        # 4. Exportação Sonarr-Ready
        $results | Select-Object Title, TvdbId, Seasons, Severity |
            Export-Csv -Path $OutputCsvPath -NoTypeInformation -Encoding UTF8

        Write-Host "✅ Protocolo de Reaquisição gerado: $OutputCsvPath" -ForegroundColor Green
    }
    finally {
        # 5. Protocolo de Higiene
        Remove-Item $tempDb -Force -ErrorAction SilentlyContinue
        Write-Host "🧹 Câmara de Isolamento limpa." -ForegroundColor DarkGray
    }
}

# Inicialização
$PlexDB = Get-PlexDatabasePath
Write-Host "Linfócito de Integridade operando em: $PlexDB" -ForegroundColor Cyan
