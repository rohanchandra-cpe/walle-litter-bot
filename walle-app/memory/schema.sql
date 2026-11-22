-- WALL-E memory store schema, version 1 (ROADMAP Phase 2). Explained in README.md.
-- Positions are in the park frame (x forward from the start, y left; m, deg).
-- Times are UTC ISO-8601 strings. Every table carries `source` ('robot' or 'sim'); the two
-- sources also live in separate files.

CREATE TABLE meta (
    key   TEXT PRIMARY KEY,
    value TEXT NOT NULL
);

CREATE TABLE runs (
    run_id            TEXT PRIMARY KEY,
    source            TEXT NOT NULL,
    kind              TEXT NOT NULL,              -- route | fake | (later) check_bin ...
    scenario          TEXT,
    started_utc       TEXT NOT NULL,
    ended_utc         TEXT,
    status            TEXT,                       -- running | done | aborted
    reason            TEXT,
    laps              INTEGER,
    distance_m        REAL,
    end_error_m       REAL,                       -- fused EKF estimate vs the start
    measured_error_cm REAL,                       -- tape measure (robot)
    run_dir           TEXT,                       -- relative to the repository root
    layout_json       TEXT,                       -- the park layout the run used
    notes             TEXT
);

CREATE TABLE detections (
    id          INTEGER PRIMARY KEY,
    source      TEXT NOT NULL,
    run_id      TEXT NOT NULL REFERENCES runs(run_id),
    t_utc       TEXT NOT NULL,
    x           REAL, y REAL, heading_deg REAL,   -- robot pose when the photo was taken
    label       TEXT NOT NULL,                    -- e.g. plastic_bottle
    category    TEXT NOT NULL,                    -- trash | recycling | unknown
    confidence  REAL,
    bbox_json   TEXT,                             -- [x1, y1, x2, y2] in pixels
    obj_x       REAL, obj_y REAL,                 -- estimated object position, if known
    place       TEXT NOT NULL,                    -- route | on_bin | around_bin | obstacle
    bin_id      INTEGER,                          -- for on_bin / around_bin
    bin_type    TEXT,                             -- the bin's type when seen
    photo       TEXT,                             -- relative to the repository root
    model       TEXT
);
CREATE INDEX detections_label ON detections(label);
CREATE INDEX detections_bin ON detections(bin_id, place);

CREATE TABLE bin_checks (
    id             INTEGER PRIMARY KEY,
    source         TEXT NOT NULL,
    run_id         TEXT NOT NULL REFERENCES runs(run_id),
    t_utc          TEXT NOT NULL,
    bin_id         INTEGER NOT NULL,
    bin_type       TEXT,
    x              REAL, y REAL, heading_deg REAL,
    overflow_level INTEGER,                       -- 0 = nothing above the rim ... 3; NULL = not assessed
    on_top_count   INTEGER,                       -- items seen above the rim
    litter_count   INTEGER,                       -- items on the ground around the bin
    outcome        TEXT,                          -- overflowing | litter | clean | NULL
    photo          TEXT,
    method         TEXT NOT NULL                  -- photo_only (Phase 2) | tilt | level_only (Phase 4)
);
CREATE INDEX bin_checks_bin ON bin_checks(bin_id);

CREATE TABLE obstacles (
    id             INTEGER PRIMARY KEY,
    source         TEXT NOT NULL,
    first_seen_utc TEXT NOT NULL,
    first_run_id   TEXT NOT NULL REFERENCES runs(run_id),
    side           INTEGER,                       -- route side 1-4
    obj_x          REAL, obj_y REAL,
    label          TEXT,
    category       TEXT
);

-- History of each obstacle. It is on the route now if its latest sighting has present = 1.
CREATE TABLE obstacle_sightings (
    id           INTEGER PRIMARY KEY,
    source       TEXT NOT NULL,
    obstacle_id  INTEGER NOT NULL REFERENCES obstacles(id),
    run_id       TEXT NOT NULL REFERENCES runs(run_id),
    t_utc        TEXT NOT NULL,
    present      INTEGER NOT NULL,                -- 1 = seen there, 0 = passed and not there
    x            REAL, y REAL, heading_deg REAL,
    detection_id INTEGER REFERENCES detections(id),
    action       TEXT                             -- passed | stopped | NULL
);
CREATE INDEX sightings_obstacle ON obstacle_sightings(obstacle_id, t_utc);

-- Free-text recall: one sentence per record and its embedding (float32 array).
CREATE TABLE memory_text (
    id          INTEGER PRIMARY KEY,
    source      TEXT NOT NULL,
    kind        TEXT NOT NULL,                    -- run | detection | bin_check | obstacle | sighting
    ref_id      TEXT NOT NULL,                    -- id in that kind's table
    run_id      TEXT,
    t_utc       TEXT NOT NULL,
    text        TEXT NOT NULL,
    embedding   BLOB,                             -- NULL until embedded
    embed_model TEXT
);
CREATE INDEX memory_text_pending ON memory_text(embedding) WHERE embedding IS NULL;
