import socketserver
from datetime import datetime

HOST = "0.0.0.0"
PORT = 8080
OUTFILE = f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"

class Handler(socketserver.BaseRequestHandler):
    def handle(self):
        print(f"[+] Conexion desde {self.client_address}")
        buffer = b""
        with open(OUTFILE, "a", encoding="utf-8") as f:
            while True:
                data = self.request.recv(1024)
                if not data:
                    break
                buffer += data
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    text = line.decode("utf-8", errors="replace").strip()
                    if text:
                        print(f"[{self.client_address[0]}] {text}")
                        f.write(text + "\n")
                        f.flush()
        print(f"[-] Conexion cerrada {self.client_address}")

class MyTCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True

if __name__ == "__main__":
    print(f"Guardando en: {OUTFILE}")
    with MyTCPServer((HOST, PORT), Handler) as server:
        print(f"Servidor escuchando en {HOST}:{PORT}")
        server.serve_forever()