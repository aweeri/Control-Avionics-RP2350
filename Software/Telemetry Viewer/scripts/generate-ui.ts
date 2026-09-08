import { readFileSync, writeFileSync, mkdirSync } from "fs";
import { join, dirname } from "path";
import { fileURLToPath } from "url";

const __dirname = dirname(fileURLToPath(import.meta.url));
const htmlPath = join(__dirname, "..", "public", "index.html");
const outPath = join(__dirname, "..", "generated", "ui.ts");

const html = readFileSync(htmlPath, "utf8");
// Escape backticks and ${} so the HTML can live inside a template literal.
const escaped = html.replace(/\\/g, "\\\\").replace(/`/g, "\\`").replace(/\$\{/g, "\\${");

mkdirSync(dirname(outPath), { recursive: true });
writeFileSync(outPath, `export default \`${escaped}\`;\n`);
console.log("Generated UI module ->", outPath);