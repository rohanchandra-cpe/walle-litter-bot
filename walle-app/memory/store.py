"""WALL-E memory store (ROADMAP Phase 2): everything the robot observes, kept in SQLite.

    from memory.store import MemoryStore            # with walle-app/ on sys.path
    mem = MemoryStore("robot")                      # walle-app/data/memory/robot.sqlite
    run = mem.start_run(kind="route", scenario="default", layout=layout)
    mem.add_bin_check(run, bin_id=4, overflow_level=2, on_top_count=5, litter_count=0)
    mem.add_detection(run, "can", "recycling", place="on_bin", bin_id=4, confidence=0.8)
    mem.end_run(run, status="done")
    mem.bin_overflow_ranking()

Sim and robot records are kept in separate files (one MemoryStore per source). Writers
(route follower, later the detection node and bin check) each open their own store; every
write is one short transaction, so they can write at the same time. Reads return plain
dicts and lists, ready to be sent as JSON by a ROSA tool. Schema: schema.sql, README.md.
"""

import json
import math
import os
import re
import sqlite3
import threading
from array import array
from contextlib import contextmanager
from datetime import datetime, timezone

import yaml

from memory.embed import OllamaEmbedder

MEMORY_DIR = os.path.dirname(os.path.abspath(__file__))
APP_DIR = os.path.dirname(MEMORY_DIR)
REPO_DIR = os.path.dirname(APP_DIR)
CONFIG_FILE = os.path.join(APP_DIR, "config", "memory.yaml")
SCHEMA_FILE = os.path.join(MEMORY_DIR, "schema.sql")
SCHEMA_VERSION = 1
SOURCES = ("robot", "sim")
CATEGORIES = ("trash", "recycling", "unknown")
PLACES = ("route", "on_bin", "around_bin", "obstacle")
OUTCOMES = ("overflowing", "litter", "clean")
BIN_TYPES = ("trash", "recycling")


class MemoryStoreError(ValueError):
    """A record the store refuses (bad value, unknown run, wrong source)."""


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def load_config(path=CONFIG_FILE):
    with open(path) as f:
        return yaml.safe_load(f)


def repo_relative(path):
    """Paths are stored relative to the repository root, so they work on the host and in
    the containers (/workspace)."""
    if not path:
        return path
    path = os.path.abspath(path)
    try:
        rel = os.path.relpath(path, REPO_DIR)
    except ValueError:
        return path
    return path if rel.startswith("..") else rel


def db_path(source, cfg=None):
    cfg = cfg or load_config()
    return os.path.join(APP_DIR, cfg["db_dir"], cfg["files"][source])


def _pack(vec):
    return array("f", vec).tobytes()


def _unpack(blob):
    a = array("f")
    a.frombytes(blob)
    return a


def _cosine(a, b):
    dot = sum(x * y for x, y in zip(a, b))
    na = math.sqrt(sum(x * x for x in a))
    nb = math.sqrt(sum(y * y for y in b))
    return dot / (na * nb) if na and nb else 0.0


def _pose(pose):
    if pose is None:
        return (None, None, None)
    pose = list(pose) + [None] * (3 - len(pose))
    return tuple(None if v is None else float(v) for v in pose[:3])


def _check(value, allowed, what):
    if value not in allowed:
        raise MemoryStoreError(f"{what} must be one of {allowed}, not {value!r}.")


class MemoryStore:
    def __init__(self, source, path=None, embedder="config", cfg=None):
        _check(source, SOURCES, "source")
        self.source = source
        self.cfg = cfg or load_config()
        self.path = path or db_path(source, self.cfg)
        if embedder == "config":
            e = self.cfg.get("embed", {})
            embedder = (OllamaEmbedder(e.get("model", "nomic-embed-text"), e.get("urls", []),
                                       e.get("timeout_s", 5.0)) if e.get("enabled") else None)
        self.embedder = embedder
        self.embed_on_write = self.cfg.get("embed", {}).get("on_write", True)
        self._lock = threading.Lock()
        os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)
        self.db = sqlite3.connect(self.path, timeout=float(self.cfg.get("busy_timeout_s", 5)),
                                  isolation_level=None, check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        # Rollback journal, not WAL: WAL's shared-memory file is unreliable on Docker
        # Desktop bind mounts, and the writes here are small and rare.
        self.db.execute("PRAGMA journal_mode=DELETE")
        self.db.execute("PRAGMA foreign_keys=ON")
        self.db.execute(f"PRAGMA busy_timeout={int(1000 * self.cfg.get('busy_timeout_s', 5))}")
        try:
            self._init_schema()
        except BaseException:
            self.db.close()
            raise

    # ----- plumbing -----
    def _init_schema(self):
        with self._tx() as db:
            version = db.execute("PRAGMA user_version").fetchone()[0]
            if version == 0:
                with open(SCHEMA_FILE) as f:
                    sql = re.sub(r"--[^\n]*", "", f.read())     # comments may hold ';'
                for stmt in sql.split(";"):
                    if stmt.strip():
                        db.execute(stmt)
                db.execute("INSERT INTO meta VALUES ('source', ?)", (self.source,))
                db.execute(f"PRAGMA user_version={SCHEMA_VERSION}")
            elif version > SCHEMA_VERSION:
                raise MemoryStoreError(f"{self.path} has schema version {version}; this code "
                                   f"knows up to {SCHEMA_VERSION}.")
            stored = db.execute("SELECT value FROM meta WHERE key='source'").fetchone()[0]
        if stored != self.source:
            raise MemoryStoreError(f"{self.path} holds {stored} records, not {self.source}.")

    @contextmanager
    def _tx(self):
        with self._lock:
            self.db.execute("BEGIN IMMEDIATE")
            try:
                yield self.db
                self.db.execute("COMMIT")
            except BaseException:
                self.db.execute("ROLLBACK")
                raise

    def _rows(self, sql, args=()):
        with self._lock:
            return [dict(r) for r in self.db.execute(sql, args).fetchall()]

    def close(self):
        self.db.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _run(self, db, run_id):
        row = db.execute("SELECT * FROM runs WHERE run_id=?", (run_id,)).fetchone()
        if row is None:
            raise MemoryStoreError(f"no run '{run_id}' in the {self.source} store; start_run first.")
        return row

    def _bin_type(self, db, run_id, bin_id):
        """The bin's type from the run's layout (None if the run has no layout)."""
        layout = self._run(db, run_id)["layout_json"]
        if not layout:
            return None
        for b in json.loads(layout).get("bins", []):
            if b.get("id") == bin_id:
                return b.get("type")
        return None

    def _remember(self, db, kind, ref_id, run_id, t_utc, text):
        """Add the sentence for a record; its embedding is added after the transaction."""
        cur = db.execute("INSERT INTO memory_text (source, kind, ref_id, run_id, t_utc, text) "
                         "VALUES (?, ?, ?, ?, ?, ?)",
                         (self.source, kind, str(ref_id), run_id, t_utc, text))
        return cur.lastrowid

    def _embed_rows(self, ids):
        if not ids or self.embedder is None or not self.embed_on_write:
            return
        self.embed_pending(ids=ids)

    # ----- writes -----
    def has_run(self, run_id):
        with self._lock:
            return self.db.execute("SELECT 1 FROM runs WHERE run_id=?",
                                   (run_id,)).fetchone() is not None

    def start_run(self, run_id=None, kind="route", scenario=None, layout=None,
                  started_utc=None, run_dir=None, laps=None, notes=None):
        """Record a new run (status 'running'); returns its run id."""
        started_utc = started_utc or utc_now()
        run_id = run_id or datetime.now().strftime("%Y%m%d_%H%M%S")
        with self._tx() as db:
            if db.execute("SELECT 1 FROM runs WHERE run_id=?", (run_id,)).fetchone():
                raise MemoryStoreError(f"run '{run_id}' is already in the {self.source} store.")
            db.execute("INSERT INTO runs (run_id, source, kind, scenario, started_utc, status, "
                       "laps, run_dir, layout_json, notes) VALUES (?,?,?,?,?,?,?,?,?,?)",
                       (run_id, self.source, kind, scenario, started_utc, "running", laps,
                        repo_relative(run_dir), json.dumps(layout) if layout else None, notes))
        return run_id

    def end_run(self, run_id, status="done", ended_utc=None, reason=None, distance_m=None,
                end_error_m=None, measured_error_cm=None, notes=None):
        ended_utc = ended_utc or utc_now()
        with self._tx() as db:
            run = self._run(db, run_id)
            db.execute("UPDATE runs SET status=?, ended_utc=?, reason=?, distance_m=?, "
                       "end_error_m=?, measured_error_cm=?, notes=COALESCE(?, notes) "
                       "WHERE run_id=?", (status, ended_utc, reason, distance_m, end_error_m,
                                          measured_error_cm, notes, run_id))
            parts = [f"{run['kind'].capitalize()} run {run_id} ({self.source}"
                     + (f", scenario {run['scenario']}" if run["scenario"] else "") + f"): {status}"]
            if reason:
                parts.append(f"({reason})")
            if distance_m is not None:
                parts.append(f"drove {distance_m:.2f} m")
            if end_error_m is not None:
                parts.append(f"ended {end_error_m * 100:.1f} cm from the start (estimate)")
            if measured_error_cm is not None:
                parts.append(f"{measured_error_cm:.1f} cm by tape")
            mid = self._remember(db, "run", run_id, run_id, ended_utc, ", ".join(parts) + ".")
        self._embed_rows([mid])

    def add_detection(self, run_id, label, category, place="route", confidence=None,
                      pose=None, obj_xy=None, bin_id=None, bbox=None, photo=None,
                      model=None, t_utc=None):
        """One object seen in one photo. place: route | on_bin (above a bin's rim) |
        around_bin (on the ground near it) | obstacle. Returns the detection id."""
        _check(category, CATEGORIES, "category")
        _check(place, PLACES, "place")
        if place in ("on_bin", "around_bin") and bin_id is None:
            raise MemoryStoreError(f"a detection {place} needs a bin_id.")
        t_utc = t_utc or utc_now()
        x, y, h = _pose(pose)
        ox, oy = (None, None) if obj_xy is None else (float(obj_xy[0]), float(obj_xy[1]))
        with self._tx() as db:
            bin_type = self._bin_type(db, run_id, bin_id) if bin_id is not None else None
            cur = db.execute(
                "INSERT INTO detections (source, run_id, t_utc, x, y, heading_deg, label, "
                "category, confidence, bbox_json, obj_x, obj_y, place, bin_id, bin_type, "
                "photo, model) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
                (self.source, run_id, t_utc, x, y, h, label, category, confidence,
                 json.dumps(bbox) if bbox else None, ox, oy, place, bin_id, bin_type,
                 repo_relative(photo), model))
            det_id = cur.lastrowid
            where = {"route": "on the route", "obstacle": "as an obstacle on the route",
                     "on_bin": f"on top of bin {bin_id}", "around_bin": f"next to bin {bin_id}"}
            text = (f"Saw a {label.replace('_', ' ')} ({category}"
                    + (f", confidence {confidence:.2f}" if confidence is not None else "")
                    + f") {where[place]}"
                    + (f" (a {bin_type} bin)" if bin_type else "")
                    + (f" at ({ox:.2f}, {oy:.2f})" if ox is not None else "")
                    + f" in run {run_id}.")
            mid = self._remember(db, "detection", det_id, run_id, t_utc, text)
        self._embed_rows([mid])
        return det_id

    def add_bin_check(self, run_id, bin_id, overflow_level=None, on_top_count=None,
                      litter_count=None, outcome=None, pose=None, photo=None,
                      method="photo_only", t_utc=None):
        """One look at one bin. overflow_level 0 (nothing above the rim) ... 3, or None if
        not assessed. outcome is derived when left out and the counts are known:
        overflowing if level > 0, else litter if litter_count > 0, else clean."""
        if overflow_level is not None and not 0 <= int(overflow_level) <= 3:
            raise MemoryStoreError(f"overflow_level must be 0-3, not {overflow_level}.")
        if outcome is None and overflow_level is not None:
            if overflow_level > 0:
                outcome = "overflowing"
            elif litter_count is not None:
                outcome = "litter" if litter_count > 0 else "clean"
        if outcome is not None:
            _check(outcome, OUTCOMES, "outcome")
        t_utc = t_utc or utc_now()
        x, y, h = _pose(pose)
        with self._tx() as db:
            bin_type = self._bin_type(db, run_id, bin_id)
            cur = db.execute(
                "INSERT INTO bin_checks (source, run_id, t_utc, bin_id, bin_type, x, y, "
                "heading_deg, overflow_level, on_top_count, litter_count, outcome, photo, "
                "method) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
                (self.source, run_id, t_utc, bin_id, bin_type, x, y, h, overflow_level,
                 on_top_count, litter_count, outcome, repo_relative(photo), method))
            check_id = cur.lastrowid
            name = f"Bin {bin_id}" + (f" ({bin_type})" if bin_type else "")
            if overflow_level is None:
                text = f"{name} photographed in run {run_id}; not assessed ({method})."
            elif outcome == "overflowing":
                text = (f"{name} was overflowing, level {overflow_level} of 3"
                        + (f", {on_top_count} items above the rim" if on_top_count else "")
                        + (f", {litter_count} litter items around it" if litter_count else "")
                        + f", in run {run_id}.")
            elif outcome == "litter":
                text = (f"{name} was not overflowing but had {litter_count} litter items "
                        f"around it in run {run_id}.")
            else:
                text = f"{name} was fine (not overflowing, no litter) in run {run_id}."
            mid = self._remember(db, "bin_check", check_id, run_id, t_utc, text)
        self._embed_rows([mid])
        return check_id

    def report_obstacle(self, run_id, label, category, obj_xy, side=None, pose=None,
                        detection_id=None, action=None, t_utc=None):
        """An obstacle seen on the route. Matches an obstacle still on the route within
        obstacle_match_radius_m (same side if both have one), or records a new one; adds a
        'present' sighting either way. Returns the obstacle id."""
        _check(category, CATEGORIES, "category")
        t_utc = t_utc or utc_now()
        ox, oy = float(obj_xy[0]), float(obj_xy[1])
        radius = float(self.cfg.get("obstacle_match_radius_m", 0.10))
        match = None
        for o in self.current_obstacles():
            if side is not None and o["side"] is not None and o["side"] != side:
                continue
            d = math.hypot(o["position"][0] - ox, o["position"][1] - oy)
            if d <= radius and (match is None or d < match[0]):
                match = (d, o["id"])
        x, y, h = _pose(pose)
        with self._tx() as db:
            self._run(db, run_id)
            if match:
                obs_id = match[1]
                text = (f"The {label.replace('_', ' ')} obstacle #{obs_id} was still on the "
                        f"route at ({ox:.2f}, {oy:.2f}) in run {run_id}.")
            else:
                obs_id = db.execute(
                    "INSERT INTO obstacles (source, first_seen_utc, first_run_id, side, obj_x, "
                    "obj_y, label, category) VALUES (?,?,?,?,?,?,?,?)",
                    (self.source, t_utc, run_id, side, ox, oy, label, category)).lastrowid
                text = (f"New obstacle #{obs_id}: a {label.replace('_', ' ')} ({category}) on "
                        + (f"side {side} of " if side else "")
                        + f"the route at ({ox:.2f}, {oy:.2f}), run {run_id}.")
            sid = db.execute(
                "INSERT INTO obstacle_sightings (source, obstacle_id, run_id, t_utc, present, x, "
                "y, heading_deg, detection_id, action) VALUES (?,?,?,?,1,?,?,?,?,?)",
                (self.source, obs_id, run_id, t_utc, x, y, h, detection_id, action)).lastrowid
            mid = self._remember(db, "sighting", sid, run_id, t_utc, text)
        self._embed_rows([mid])
        return obs_id

    def mark_absent(self, run_id, obstacle_id, pose=None, t_utc=None):
        """The robot passed where the obstacle was and it is gone: it is cleared."""
        t_utc = t_utc or utc_now()
        x, y, h = _pose(pose)
        with self._tx() as db:
            self._run(db, run_id)
            o = db.execute("SELECT * FROM obstacles WHERE id=?", (obstacle_id,)).fetchone()
            if o is None:
                raise MemoryStoreError(f"no obstacle #{obstacle_id}.")
            sid = db.execute(
                "INSERT INTO obstacle_sightings (source, obstacle_id, run_id, t_utc, present, x, "
                "y, heading_deg) VALUES (?,?,?,?,0,?,?,?)",
                (self.source, obstacle_id, run_id, t_utc, x, y, h)).lastrowid
            mid = self._remember(db, "sighting", sid, run_id, t_utc,
                                 f"Obstacle #{obstacle_id} ({o['label']}) was gone (cleared) "
                                 f"in run {run_id}.")
        self._embed_rows([mid])

    def embed_pending(self, ids=None, batch=32):
        """Embed stored text that has no embedding yet. Returns how many were embedded."""
        if self.embedder is None:
            return 0
        sql = "SELECT id, text FROM memory_text WHERE embedding IS NULL"
        args = ()
        if ids is not None:
            sql += f" AND id IN ({','.join('?' * len(ids))})"
            args = tuple(ids)
        rows = self._rows(sql, args)
        done = 0
        for i in range(0, len(rows), batch):
            chunk = rows[i:i + batch]
            vecs = self.embedder.embed([r["text"] for r in chunk])
            if not vecs:
                break
            with self._tx() as db:
                for r, v in zip(chunk, vecs):
                    db.execute("UPDATE memory_text SET embedding=?, embed_model=? WHERE id=?",
                               (_pack(v), self.embedder.model, r["id"]))
            done += len(chunk)
        return done

    # ----- reads (what the agent asks) -----
    def runs(self, limit=10, kind=None):
        sql = ("SELECT run_id, kind, scenario, started_utc, ended_utc, status, reason, laps, "
               "distance_m, end_error_m, measured_error_cm, run_dir FROM runs")
        args = []
        if kind:
            sql += " WHERE kind=?"
            args.append(kind)
        sql += " ORDER BY started_utc DESC LIMIT ?"
        return self._rows(sql, args + [limit])

    def most_seen(self, limit=5, since=None, place=None):
        """Objects by how often they were seen: sightings, runs they were seen in, their
        category (most common), and when last seen."""
        where, args = [], []
        if since:
            where.append("t_utc >= ?")
            args.append(since)
        if place:
            where.append("place = ?")
            args.append(place)
        w = (" WHERE " + " AND ".join(where)) if where else ""
        rows = self._rows(
            "SELECT label, COUNT(*) AS times_seen, COUNT(DISTINCT run_id) AS runs_seen, "
            f"MAX(t_utc) AS last_seen_utc FROM detections{w} GROUP BY label "
            "ORDER BY times_seen DESC, label LIMIT ?", args + [limit])
        for r in rows:
            cats = self._rows(f"SELECT category, COUNT(*) AS n FROM detections{w}"
                              + (" AND" if w else " WHERE") + " label=? GROUP BY category "
                              "ORDER BY n DESC", args + [r["label"]])
            r["category"] = cats[0]["category"] if cats else None
        return rows

    def bin_overflow_ranking(self):
        """Per bin, over all checks that assessed it: how often it overflowed, the average
        and highest level when it did, how often it had litter around it, and how often it
        was fine. Most often overflowing first."""
        rows = self._rows(
            "SELECT bin_id, MAX(bin_type) AS bin_type, COUNT(*) AS checks, "
            "SUM(overflow_level IS NOT NULL) AS assessed, "
            "SUM(overflow_level > 0) AS times_overflowing, "
            "AVG(CASE WHEN overflow_level > 0 THEN overflow_level END) AS mean_level, "
            "MAX(overflow_level) AS max_level, "
            "SUM(litter_count > 0) AS times_litter, "
            "SUM(outcome = 'clean') AS times_clean, MAX(t_utc) AS last_check_utc "
            "FROM bin_checks GROUP BY bin_id")
        for r in rows:
            r["times_overflowing"] = r["times_overflowing"] or 0
            r["times_litter"] = r["times_litter"] or 0
            r["times_clean"] = r["times_clean"] or 0
            r["overflow_rate"] = (round(r["times_overflowing"] / r["assessed"], 3)
                                  if r["assessed"] else None)
            if r["mean_level"] is not None:
                r["mean_level"] = round(r["mean_level"], 2)
        rows.sort(key=lambda r: (-(r["overflow_rate"] or 0), -(r["mean_level"] or 0),
                                 r["bin_id"]))
        return rows

    def last_bin_check(self, bin_id):
        rows = self._rows("SELECT * FROM bin_checks WHERE bin_id=? ORDER BY t_utc DESC, id "
                          "DESC LIMIT 1", (bin_id,))
        return rows[0] if rows else None

    def bin_check_items(self, run_id, bin_id):
        """What a run saw at one bin: the items above its rim and the litter around it
        (labels, categories, confidences), for reporting a bin check (Phase 6)."""
        rows = self._rows("SELECT label, category, place, confidence FROM detections "
                          "WHERE run_id=? AND bin_id=? AND place IN ('on_bin', 'around_bin') "
                          "ORDER BY place DESC, confidence DESC", (run_id, bin_id))
        return {"on_top": [r for r in rows if r["place"] == "on_bin"],
                "around": [r for r in rows if r["place"] == "around_bin"]}

    def current_obstacles(self):
        """Obstacles whose latest sighting says they are still there."""
        rows = self._rows(
            "SELECT o.*, s.t_utc AS last_seen_utc, s.run_id AS last_run_id, "
            "(SELECT COUNT(*) FROM obstacle_sightings c WHERE c.obstacle_id=o.id AND "
            " c.present=1) AS times_seen "
            "FROM obstacles o JOIN obstacle_sightings s ON s.id = ("
            "  SELECT id FROM obstacle_sightings WHERE obstacle_id=o.id "
            "  ORDER BY t_utc DESC, id DESC LIMIT 1) "
            "WHERE s.present = 1 ORDER BY o.side, o.id")
        for r in rows:
            r["position"] = [r.pop("obj_x"), r.pop("obj_y")]
        return rows

    def obstacle_history(self, obstacle_id):
        return self._rows("SELECT * FROM obstacle_sightings WHERE obstacle_id=? "
                          "ORDER BY t_utc, id", (obstacle_id,))

    def recycling_correctness(self):
        """Are items going into the right bins? Judged ONLY by items seen on top of a bin
        (above the rim: the robot can't see inside the opaque bins): an item counts as put
        in that bin, correct when its category matches the bin's type. Items of unknown
        category are counted apart. Litter around the bins is reported separately."""
        on_top = self._rows("SELECT label, category, bin_id, bin_type FROM detections "
                            "WHERE place='on_bin'")
        per_bin, wrong = {}, {}
        total = {"correct": 0, "wrong": 0, "unclassified": 0}
        for d in on_top:
            b = per_bin.setdefault(d["bin_id"], {"bin_id": d["bin_id"],
                                                 "bin_type": d["bin_type"],
                                                 "correct": 0, "wrong": 0, "unclassified": 0})
            if d["category"] == "unknown" or d["bin_type"] not in BIN_TYPES:
                key = "unclassified"
            elif d["category"] == d["bin_type"]:
                key = "correct"
            else:
                key = "wrong"
                k = (d["label"], d["category"], d["bin_id"], d["bin_type"])
                wrong[k] = wrong.get(k, 0) + 1
            b[key] += 1
            total[key] += 1

        def rate(c):
            judged = c["correct"] + c["wrong"]
            return round(c["correct"] / judged, 3) if judged else None

        for b in per_bin.values():
            b["correct_rate"] = rate(b)
        total["correct_rate"] = rate(total)
        around = self._rows("SELECT bin_id, MAX(bin_type) AS bin_type, category, COUNT(*) AS n "
                            "FROM detections WHERE place='around_bin' "
                            "GROUP BY bin_id, category ORDER BY bin_id, category")
        return {
            "overall": total,
            "per_bin": [per_bin[k] for k in sorted(per_bin)],
            "wrong_items": [{"label": l, "category": c, "bin_id": b, "bin_type": t, "times": n}
                            for (l, c, b, t), n in sorted(wrong.items(),
                                                          key=lambda kv: -kv[1])],
            "litter_around_bins": around,
            "basis": "Only items visible above a bin's rim are judged (the bins are opaque); "
                     "a bin that is not overflowing gives no evidence. Litter on the ground "
                     "is listed separately and not judged.",
        }

    def recall(self, query, k=None):
        """Stored records most related to a free-text question: by embedding similarity,
        or by shared keywords when no embeddings are available."""
        k = k or int(self.cfg.get("recall_k", 5))
        rows = self._rows("SELECT id, kind, ref_id, run_id, t_utc, text, embedding "
                          "FROM memory_text")
        qv = None
        if self.embedder is not None and any(r["embedding"] for r in rows):
            vecs = self.embedder.embed([query])
            qv = vecs[0] if vecs else None
        scored = []
        if qv is not None:
            for r in rows:
                if r["embedding"]:
                    scored.append((_cosine(qv, _unpack(r["embedding"])), r))
            method = "vector"
        else:
            words = set(re.findall(r"[a-z0-9]+", query.lower())) - _STOP
            for r in rows:
                have = set(re.findall(r"[a-z0-9]+", r["text"].lower()))
                hits = len(words & have)
                if hits:
                    scored.append((hits / len(words), r))
            method = "keyword"
        scored.sort(key=lambda sr: (-sr[0], sr[1]["t_utc"]))
        out = []
        for s, r in scored[:k]:
            r = dict(r)
            r.pop("embedding")
            r["score"] = round(s, 3)
            r["method"] = method
            out.append(r)
        return out

    def counts(self):
        """Row counts per table (for checks and status)."""
        tables = ("runs", "detections", "bin_checks", "obstacles", "obstacle_sightings",
                  "memory_text")
        out = {t: self._rows(f"SELECT COUNT(*) AS n FROM {t}")[0]["n"] for t in tables}
        out["unembedded"] = self._rows("SELECT COUNT(*) AS n FROM memory_text "
                                       "WHERE embedding IS NULL")[0]["n"]
        return out


_STOP = {"a", "an", "the", "is", "are", "was", "were", "of", "in", "on", "at", "to", "and",
         "or", "what", "which", "where", "when", "how", "did", "do", "does", "you", "i",
         "have", "has", "any", "it", "that", "this", "for", "with", "most", "often"}
