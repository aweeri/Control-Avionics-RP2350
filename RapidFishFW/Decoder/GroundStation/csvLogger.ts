import { mkdirSync, appendFileSync, writeFileSync, rmSync } from "fs";
import { join } from "path";

function escapeCSV(field: unknown): string {
  if (field === null || field === undefined) return "";
  const s = String(field);
  if (/[",\n\r]/.test(s)) return '"' + s.replace(/"/g, '""') + '"';
  return s;
}

/** Flatten nested object into dot-notation keys (arrays become [i]). */
export function flatten(obj: Record<string, unknown>, prefix = ""): Record<string, unknown> {
  const out: Record<string, unknown> = {};
  for (const [k, v] of Object.entries(obj)) {
    const key = prefix ? `${prefix}.${k}` : k;
    if (v && typeof v === "object" && !Array.isArray(v)) {
      Object.assign(out, flatten(v as Record<string, unknown>, key));
    } else if (Array.isArray(v)) {
      v.forEach((item, i) => {
        if (item && typeof item === "object") Object.assign(out, flatten(item as Record<string, unknown>, `${key}[${i}]`));
        else out[`${key}[${i}]`] = item;
      });
    } else {
      out[key] = v;
    }
  }
  return out;
}

function timestamp(): string {
  const d = new Date();
  return d.toISOString().replace(/[:T]/g, "-").replace(/\.\d{3}Z$/, "");
}

function sanitizeApid(apid: unknown): string {
  if (apid === undefined || apid === null) return "unknown";
  return String(apid).replace(/[^a-zA-Z0-9_]/g, "_");
}

/** One dynamic-column CSV file for a single apid. */
class ApidLogger {
  readonly path: string;
  private columns = new Set<string>();
  private rows: Record<string, unknown>[] = [];
  private headerWritten = false;

  constructor(dir: string, apid: string) {
    mkdirSync(dir, { recursive: true });
    this.path = join(dir, `apid-${apid}.csv`);
    writeFileSync(this.path, "");
  }

  add(obj: Record<string, unknown>) {
    const flat = flatten(obj);
    let newColumns = false;
    for (const key of Object.keys(flat)) {
      if (!this.columns.has(key)) {
        this.columns.add(key);
        newColumns = true;
      }
    }
    this.rows.push(flat);

    if (newColumns) {
      this.rewriteAll();
    } else {
      const cols = Array.from(this.columns);
      const line = cols.map((c) => escapeCSV(flat[c])).join(",");
      this.appendHeaderIfNeeded();
      appendFileSync(this.path, line + "\n");
    }
  }

  flush() {
    this.rewriteAll();
  }

  getColumns(): string[] {
    return Array.from(this.columns);
  }

  private appendHeaderIfNeeded() {
    if (this.headerWritten) return;
    this.headerWritten = true;
    appendFileSync(this.path, Array.from(this.columns).join(",") + "\n");
  }

  private rewriteAll() {
    const cols = Array.from(this.columns);
    this.headerWritten = true;
    const header = cols.join(",") + "\n";
    const lines = this.rows.map((r) => cols.map((c) => escapeCSV(r[c])).join(",")).join("\n");
    writeFileSync(this.path, header + (lines ? lines + "\n" : ""));
  }
}

/**
 * Session CSV logger. Creates a per-apid CSV file inside a session subdirectory
 * of the log root, e.g. logs/rapidfish-<timestamp>/apid-0.csv. Each apid file
 * uses a dynamic (union of seen columns) schema.
 */
export class CSVLogger {
  readonly sessionDir: string;
  readonly root: string;
  private loggers = new Map<string, ApidLogger>();

  constructor(root: string, sessionDirName?: string) {
    this.root = root;
    this.sessionDir = join(root, sessionDirName ?? `rapidfish-${timestamp()}`);
    mkdirSync(this.sessionDir, { recursive: true });
  }

  add(obj: Record<string, unknown>) {
    const apid = sanitizeApid(obj.apid);
    let logger = this.loggers.get(apid);
    if (!logger) {
      logger = new ApidLogger(this.sessionDir, apid);
      this.loggers.set(apid, logger);
    }
    logger.add(obj);
  }

  flush() {
    for (const logger of this.loggers.values()) logger.flush();
  }

  /** Full paths to every per-apid CSV written this session. */
  get files(): string[] {
    return Array.from(this.loggers.values()).map((l) => l.path);
  }

  /** Clear all session data (e.g. on reconnect). */
  reset() {
    rmSync(this.sessionDir, { recursive: true, force: true });
    this.loggers.clear();
    mkdirSync(this.sessionDir, { recursive: true });
  }
}