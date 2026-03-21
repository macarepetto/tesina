#!/usr/bin/env python3
import socketserver
import threading
import json
from datetime import datetime
from http.server import BaseHTTPRequestHandler, HTTPServer

HOST = "0.0.0.0"
TCP_PORT = 8080
WEB_PORT = 8081

OUTFILE = f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
MAX_LINES = 300

lock = threading.Lock()
last_lines = []  # lista de dicts: {ts, ip, text, parsed_ok}

def add_line(ip: str, text: str):
    ts = datetime.now().strftime("%H:%M:%S")

    # Para la web: intentamos parsear (si el JSON tiene comas dobles, parsed_ok=False)
    parsed_ok = True
    try:
        json.loads(text)
    except Exception:
        parsed_ok = False

    entry = {"ts": ts, "ip": ip, "text": text, "parsed_ok": parsed_ok}

    # buffer para web
    with lock:
        last_lines.append(entry)
        if len(last_lines) > MAX_LINES:
            last_lines.pop(0)

    # consola
    print(f"[{ip}] {text}")

    # archivo: EXACTO como tu script (solo la línea)
    with open(OUTFILE, "a", encoding="utf-8") as f:
        f.write(text + "\n")
        f.flush()


class Handler(socketserver.BaseRequestHandler):
    def handle(self):
        ip, port = self.client_address
        print(f"[+] Conexion desde {ip}:{port}")
        buffer = b""

        while True:
            data = self.request.recv(1024)
            if not data:
                break

            buffer += data
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                text = line.decode("utf-8", errors="replace").strip()
                if text:
                    add_line(ip, text)

        print(f"[-] Conexion cerrada {ip}:{port}")


class MyTCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True


class WebHandler(BaseHTTPRequestHandler):
    def _send(self, code: int, body: bytes, content_type="text/html; charset=utf-8"):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/?"):
            html = f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Telemetría ESP32</title>
  <style>
    body {{ font-family: system-ui, -apple-system, Segoe UI, Roboto, Arial; margin: 16px; }}
    .top {{ display:flex; gap:12px; align-items:center; flex-wrap:wrap; }}
    .pill {{ padding: 4px 10px; background:#f2f2f2; border-radius:999px; font-size:12px; }}
    .row {{ padding: 10px 0; border-bottom: 1px solid #eee; }}
    .ip {{ color: #555; font-size: 12px; }}
    pre {{ white-space: pre-wrap; word-break: break-word; margin: 6px 0 0; }}
    .ok {{ color: #0a7; font-weight: 600; }}
    .bad {{ color: #c33; font-weight: 600; }}
  </style>
</head>
<body>
  <div class="top">
    <h2 style="margin:0;">Telemetría ESP32</h2>
    <span class="pill">TCP:{TCP_PORT} · WEB:{WEB_PORT}</span>
    <span class="pill">guardando en: {OUTFILE}</span>
  </div>
  <p>Se actualiza cada 1s. “PARSEADO” es solo para debug (el archivo siempre guarda la línea tal cual).</p>
  <div id="list"></div>

<script>
async function tick() {{
  const r = await fetch('/lines');
  const data = await r.json();
  let html = '';
  for (const item of data.lines) {{
    const badge = item.parsed_ok ? '<span class="ok">PARSEADO</span>' : '<span class="bad">NO PARSEADO</span>';
    html += `
      <div class="row">
        <div class="ip"><b>${{item.ts}}</b> · ${{item.ip}} · ${{badge}}</div>
        <pre>${{item.text}}</pre>
      </div>`;
  }}
  document.getElementById('list').innerHTML = html;
}}
setInterval(tick, 1000);
tick();
</script>
</body>
</html>"""
            self._send(200, html.encode("utf-8"))
            return

        if self.path.startswith("/lines"):
            with lock:
                payload = {"lines": list(last_lines)}
            self._send(200, json.dumps(payload).encode("utf-8"),
                       "application/json; charset=utf-8")
            return

        self._send(404, b"Not found", "text/plain; charset=utf-8")

    def log_message(self, format, *args):
        return


def run_web():
    httpd = HTTPServer((HOST, WEB_PORT), WebHandler)
    print(f"[WEB] Abrí en tu celu: http://<IP_NOTEBOOK>:{WEB_PORT}")
    httpd.serve_forever()


if __name__ == "__main__":
    print(f"Guardando en: {OUTFILE}")
    tcp = MyTCPServer((HOST, TCP_PORT), Handler)
    t = threading.Thread(target=tcp.serve_forever, daemon=True)
    t.start()

    print(f"[TCP] Servidor escuchando en {HOST}:{TCP_PORT}")
    run_web()