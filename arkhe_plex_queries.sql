-- Query for TV Shows
-- Extracts Series Title, Guid (with TVDB ID), Season, Episode and File Path
SELECT
    series.title AS SeriesTitle,
    series.guid AS SeriesGuid,
    season."index" AS SeasonNumber,
    md.title AS EpisodeTitle,
    mp.file AS FilePath
FROM metadata_items AS md
JOIN metadata_items AS season ON md.parent_id = season.id
JOIN metadata_items AS series ON season.parent_id = series.id
JOIN media_items AS mi ON md.id = mi.metadata_item_id
JOIN media_parts AS mp ON mi.id = mp.media_item_id
WHERE md.metadata_type = 4
  AND series.metadata_type = 2
  AND season.metadata_type = 3
  AND md.deleted_at IS NULL;

-- Query for Movies
-- Extracts Movie Title, Guid (with TMDB ID), Year and File Path
SELECT
    md.title AS MovieTitle,
    md.guid AS MovieGuid,
    md.year AS Year,
    mp.file AS FilePath
FROM metadata_items AS md
JOIN media_items AS mi ON md.id = mi.metadata_item_id
JOIN media_parts AS mp ON mi.id = mp.media_item_id
WHERE md.metadata_type = 1
  AND md.deleted_at IS NULL;
