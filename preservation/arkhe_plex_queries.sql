-- ARKHE(N) OS v2.1: QUERY DA VERDADE (IDs UNIVERSAIS)

-- TV SHOWS (SONARR)
-- Extracts Series Title, TVDB ID, Season, and Episode Path
SELECT
    m.title AS SeriesTitle,
    REPLACE(REPLACE(SUBSTR(m.guid, INSTR(m.guid, 'tvdb://') + 7), '?lang=en', ''), '?lang=pt', '') AS tvdbId,
    parent.index AS SeasonNumber,
    child.index AS EpisodeNumber,
    parts.file AS FilePath
FROM metadata_items m
JOIN metadata_items parent ON parent.parent_id = m.id
JOIN metadata_items child ON child.parent_id = parent.id
JOIN media_items mi ON mi.metadata_item_id = child.id
JOIN media_parts parts ON parts.media_item_id = mi.id
WHERE m.metadata_type = 2 -- Série
  AND child.metadata_type = 4 -- Episódio
  AND m.deleted_at IS NULL
ORDER BY SeriesTitle, SeasonNumber, EpisodeNumber;

-- MOVIES (RADARR)
-- Extracts Movie Title, TMDB ID, Year, and File Path
SELECT
    m.title AS MovieTitle,
    m.year AS Year,
    REPLACE(REPLACE(SUBSTR(m.guid, INSTR(m.guid, 'tmdb://') + 7), '?lang=en', ''), '?lang=pt', '') AS tmdbId,
    parts.file AS FilePath
FROM metadata_items m
JOIN media_items mi ON mi.metadata_item_id = m.id
JOIN media_parts parts ON parts.media_item_id = mi.id
WHERE m.metadata_type = 1 -- Filme
  AND m.deleted_at IS NULL
ORDER BY MovieTitle;
