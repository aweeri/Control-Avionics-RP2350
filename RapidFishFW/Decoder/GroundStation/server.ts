import { join, dirname } from "path";
import { fileURLToPath } from "url";
import { existsSync } from "fs";
import type { ServerWebSocket } from "bun";
import { CSVLogger } from "./csvLogger";
import indexHTML from "./generated/ui";

// Compiled exe's import.meta.url is not writable, so use CWD for logs.
const CWD = process.cwd();
const __dirname = dirname(fileURLToPath(import.meta.url));
const PUBLIC_DIR = join(__dirname, "public");
const LOG_DIR = process.env.LOG_DIR ?? join(CWD, "logs");
// Compiled exe has no disk assets; embedded HTML is used instead.
const hasDiskAssets = existsSync(join(PUBLIC_DIR, "index.html"));
const PORT = Number(process.env.PORT ?? 8765);

const logger = new CSVLogger(LOG_DIR);
let serialSource: ServerWebSocket<{ isSerialSource: boolean }> | null = null;
let recEnabled = true;
const clients = new Set<ServerWebSocket<{ isSerialSource: boolean }>>();

// Periodically persist buffered rows so memory stays bounded.
setInterval(() => logger.flush(), 2500).unref?.();

// Keep the process alive even if Bun.serve ends up on a detached event loop,
// and (critically) stop uncaught exceptions from killing a double-clicked exe
// immediately. Without this, a startup error makes the console flash and close.
process.on("uncaughtException", (err) => {
  console.error("\nUnhandled error:", err?.stack ?? err);
  console.error("The server may not have started. Press Ctrl+C to exit.");
});
setInterval(() => {}, 1 << 30); // hard keep-alive, not unref'd

// ---------------------------------------------------------------------------
// Telemetry broadcast
// ---------------------------------------------------------------------------
function parseLine(line: string) {
  line = line.trim();
  if (!line || !line.startsWith("{")) return;

  let obj: unknown;
  try {
    obj = JSON.parse(line);
  } catch {
    return; // skip partial / non-telemetry frames
  }
  if (!obj || typeof obj !== "object") return;
  const rec = obj as Record<string, unknown>;

  if (recEnabled) {
    try {
      logger.add(rec);
    } catch (e) {
      console.error("CSV write failed:", e);
    }
  }

  broadcast({ type: "telemetry", data: rec });
}

function broadcast(message: unknown) {
  const payload = JSON.stringify(message);
  for (const ws of clients) {
    if (ws.readyState === 1) ws.send(payload);
  }
}

function sessionInfo() {
  return {
    sessionDir: logger.sessionDir,
    files: logger.files,
  };
}

// HTTP + WebSocket server
const server = Bun.serve<{ isSerialSource: boolean }>({
  port: PORT,
  async fetch(req, srv) {
    const url = new URL(req.url);

    if (url.pathname === "/ws") {
      const ok = srv.upgrade(req, { data: { isSerialSource: false } });
      if (!ok) return new Response("Upgrade failed", { status: 500 });
      return new Response();
    }

    const reqPath = decodeURIComponent(url.pathname);
    const rel = reqPath === "/" ? "index.html" : reqPath.replace(/^\/+/, "");
    if (hasDiskAssets) {
      let file = Bun.file(join(PUBLIC_DIR, rel));
      if (!existsSync(join(PUBLIC_DIR, rel))) file = Bun.file(join(PUBLIC_DIR, "index.html"));
      return new Response(file);
    }
    return new Response(indexHTML, {
      headers: { "Content-Type": "text/html; charset=utf-8" },
    });
  },

  websocket: {
    open(ws) {
      clients.add(ws);
      ws.send(JSON.stringify({ type: "csv", ...sessionInfo() }));
    },
    message(ws, msg) {
      if (typeof msg !== "string") return;
      let parsed: { type: string; data?: string | boolean };
      try {
        parsed = JSON.parse(msg);
      } catch {
        return;
      }
      if (parsed.type === "serialSource") {
        if (!serialSource || serialSource === ws) {
          logger.reset();
        }
        serialSource = ws;
        ws.data = { isSerialSource: true };
        broadcast({ type: "serialState", connected: true });
        broadcast({ type: "csv", ...sessionInfo() });
        return;
      }
      if (parsed.type === "serialDisconnected") {
        ws.data = { isSerialSource: false };
        if (serialSource === ws) serialSource = null;
        logger.flush();
        broadcast({ type: "serialState", connected: false });
        return;
      }
      if (parsed.type === "line" && typeof parsed.data === "string") {
        parseLine(parsed.data);
        return;
      }
      if (parsed.type === "rec") {
        recEnabled = parsed.data !== false;
        broadcast({ type: "recState", rec: recEnabled });
        return;
      }
    },
    close(ws) {
      clients.delete(ws);
      if (ws.data?.isSerialSource) {
        serialSource = null;
        logger.flush();
        broadcast({ type: "serialState", connected: false });
      }
    },
  },
});

console.log(`\n  RapidFish GroundStation`);
console.log(`  Logs     -> ${logger.sessionDir}`);
console.log(`  UI       -> http://localhost:${PORT}\n`);

// Auto-open the browser (best-effort)
function openBrowser(url: string) {
  const target = new URL(url);
  if (process.env.NO_OPEN === "1") return;
  try {
    const platform = process.platform;
    if (platform === "win32") {
      Bun.spawn(["cmd", "/c", "start", "", target.toString()]);
    } else if (platform === "darwin") {
      Bun.spawn(["open", target.toString()]);
    } else {
      Bun.spawn(["xdg-open", target.toString()]);
    }
  } catch (e) {
    console.error("Could not auto-open browser:", e);
  }
}
openBrowser(`http://localhost:${PORT}`);