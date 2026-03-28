#!/usr/bin/env python3
import socketserver
import json
from datetime import datetime

# --- LIBRERÍAS WEB COMENTADAS ---
# import threading
# from http.server import BaseHTTPRequestHandler, HTTPServer

HOST = "0.0.0.0"
TCP_PORT = 8080

# --- PUERTO WEB COMENTADO ---
# WEB_PORT = 8081

OUTFILE = f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"

# --- VARIABLES WEB COMENTADAS ---
# MAX_LINES = 300
# lock = threading.Lock()
# last_lines = []  

def add_line(ip: str, text: str):
    # --- LÓGICA DE MEMORIA WEB COMENTADA ---
    '''
    ts = datetime.now().strftime("%H:%M:%S")
    parsed_ok = True
    try:
        json.loads(text)
    except Exception:
        parsed_ok = False

    entry = {"ts": ts, "ip": ip, "text": text, "parsed_ok": parsed_ok}

    with lock:
        last_lines.append(entry)
        if len(last_lines) > MAX_LINES:
            last_lines.pop(0)
    '''

    # Consola: Imprime súper rápido
    print(f"[{ip}] {text}")

    # Archivo: Guarda directo en el disco y hace flush al instante
    with open(OUTFILE, "a", encoding="utf-8") as f:
        f.write(text + "\n")
        f.flush()


class Handler(socketserver.BaseRequestHandler):
    def handle(self):
        ip = self.client_address[0]
        print(f"[+] Conexion desde {ip}:{self.client_address[1]}")
        buffer = ""
        while True:
            try:
                data = self.request.recv(4096)
                if not data:
                    print(f"[-] Desconexion de {ip}")
                    break
                
                buffer += data.decode("utf-8", errors="replace")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if line:
                        add_line(ip, line)
            except Exception as e:
                print(f"[!] Error con {ip}: {e}")
                break

# ==========================================
# BLOQUE DEL SERVIDOR WEB COMENTADO
# ==========================================
'''
class WebHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        # ... (código original web) ...
        pass
        
def run_web():
    httpd = HTTPServer((HOST, WEB_PORT), WebHandler)
    print(f"[WEB] Abrí en tu celu: http://<IP_NOTEBOOK>:{WEB_PORT}")
    httpd.serve_forever()
'''

if __name__ == "__main__":
    print(f"Guardando en: {OUTFILE}")
    print(f"[TCP] Servidor escuchando en {HOST}:{TCP_PORT}")
    
    # --- INICIO DE HILO WEB COMENTADO ---
    # threading.Thread(target=run_web, daemon=True).start()

    # MAGIA ACÁ: Le decimos a la clase base que libere el puerto SIEMPRE, 
    # antes de instanciar el servidor.
    socketserver.TCPServer.allow_reuse_address = True 

    # Iniciar servidor TCP exclusivamente
    with socketserver.ThreadingTCPServer((HOST, TCP_PORT), Handler) as server:
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print("\n[!] Servidor detenido por el usuario. Datos guardados con éxito.")