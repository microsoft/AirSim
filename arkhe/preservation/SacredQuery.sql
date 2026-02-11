-- ARKHE(N) OS: THE SACRED QUERY (V2.1)
-- Extração robusta de DNA Digital (TVDB/TMDB/IMDB IDs) para reaquisição automática.

-- 1. TV SHOWS (SONARR READY)
WITH series_guids AS (
    SELECT
        id,
        title,
        CASE
            WHEN guid LIKE '%thetvdb://%'
                THEN REPLACE(SUBSTR(guid, INSTR(guid, 'thetvdb://') + 10), '?lang=pt', '')
            WHEN guid LIKE '%tvdb://%'
                THEN REPLACE(SUBSTR(guid, INSTR(guid, 'tvdb://') + 7), '?lang=pt', '')
            ELSE NULL
        END AS tvdb_id
    FROM metadata_items
    WHERE metadata_type = 2           -- 2 = Série
        AND deleted_at IS NULL
        AND guid IS NOT NULL
)
SELECT
    sg.title AS SeriesTitle,
    sg.tvdb_id AS TvdbId,
    ep.parent_index AS SeasonNumber,
    ep."index" AS EpisodeNumber,
    mp.file_path AS FilePath
FROM metadata_items AS ep
JOIN series_guids AS sg ON ep.parent_id = sg.id
JOIN media_items AS mi ON ep.id = mi.metadata_item_id
JOIN media_parts AS mp ON mi.id = mp.media_item_id
WHERE ep.metadata_type = 4           -- 4 = Episódio
    AND ep.deleted_at IS NULL
    AND mp.file_path IS NOT NULL
ORDER BY sg.title, ep.parent_index, ep."index";

-- 2. MOVIES (RADARR READY)
SELECT
    md.title AS MovieTitle,
    md.year AS Year,
    CASE
        WHEN md.guid LIKE '%themoviedb://%'
            THEN REPLACE(SUBSTR(md.guid, INSTR(md.guid, 'themoviedb://') + 13), '?lang=pt', '')
        WHEN md.guid LIKE '%tmdb://%'
            THEN REPLACE(SUBSTR(md.guid, INSTR(md.guid, 'tmdb://') + 7), '?lang=pt', '')
        WHEN md.guid LIKE '%imdb://%'
            THEN REPLACE(SUBSTR(md.guid, INSTR(md.guid, 'imdb://') + 7), '?lang=pt', '')
        ELSE NULL
    END AS TmdbId,
    mp.file_path AS FilePath
FROM metadata_items AS md
JOIN media_items AS mi ON md.id = mi.metadata_item_id
JOIN media_parts AS mp ON mi.id = mp.media_item_id
WHERE md.metadata_type = 1           -- 1 = Filme
    AND md.deleted_at IS NULL
    AND md.guid IS NOT NULL
    AND mp.file_path IS NOT NULL;
