#!/usr/bin/env python3
"""Recibe telemetría JSONL de varios ESP32 y la separa por ID de dispositivo."""

import json
import re
import socketserver
import threading
from datetime import datetime
from pathlib import Path


HOST = "0.0.0.0"
TCP_PORT = 8080

# Todos los archivos de esta ejecución comparten el mismo identificador.
SESION = datetime.now().strftime("%Y%m%d_%H%M%S")
CARPETA_LOGS = Path.cwd()

# La escritura se serializa para que dos hilos no intercalen sus mensajes.
lock_archivos = threading.Lock()
archivos_anunciados = set()
origenes_por_id = {}


def nombre_dispositivo(registro: dict) -> str:
    """Obtiene un nombre seguro a partir del campo JSON 'id'."""
    identificador = registro.get("id")
    if not isinstance(identificador, str) or not identificador.strip():
        return "sin_id"
    nombre = re.sub(r"[^A-Za-z0-9_-]", "_", identificador.strip())[:80]
    return nombre or "sin_id"


def add_line(ip: str, text: str) -> None:
    """Guarda sin modificar el JSON original, en el archivo de su prototipo."""
    try:
        registro = json.loads(text)
    except json.JSONDecodeError:
        # Nunca contaminamos los archivos JSONL con líneas inválidas.
        ruta = CARPETA_LOGS / f"log_{SESION}_invalidos.txt"
        with lock_archivos:
            with ruta.open("a", encoding="utf-8") as archivo:
                archivo.write(f"[{ip}] {text}\n")
            print(f"[!] JSON inválido desde {ip}; guardado en {ruta.name}", flush=True)
        return

    dispositivo = nombre_dispositivo(registro) if isinstance(registro, dict) else "sin_id"
    ruta = CARPETA_LOGS / f"log_{SESION}_{dispositivo}.jsonl"

    with lock_archivos:
        if ruta not in archivos_anunciados:
            archivos_anunciados.add(ruta)
            print(f"[ARCHIVO] {dispositivo}: {ruta}", flush=True)

        if dispositivo == "sin_id":
            print(f"[!] Mensaje de {ip} sin campo 'id'; revisar firmware.", flush=True)
        else:
            ips = origenes_por_id.setdefault(dispositivo, set())
            if ips and ip not in ips:
                print(
                    f"[!] ATENCIÓN: {dispositivo} aparece desde varias IP: "
                    f"{', '.join(sorted(ips | {ip}))}. Revisar DEVICE_ID.",
                    flush=True,
                )
            ips.add(ip)

        with ruta.open("a", encoding="utf-8") as archivo:
            archivo.write(text + "\n")
            archivo.flush()

        # Mantenemos la salida detallada que tenía el servidor original.
        print(f"[{dispositivo} | {ip}] {text}", flush=True)


class Handler(socketserver.BaseRequestHandler):
    def handle(self) -> None:
        ip, puerto = self.client_address
        with lock_archivos:
            print(f"[+] Conexión desde {ip}:{puerto}", flush=True)

        buffer = b""
        while True:
            try:
                datos = self.request.recv(4096)
                if not datos:
                    with lock_archivos:
                        print(f"[-] Desconexión de {ip}:{puerto}", flush=True)
                    break

                # Separar en bytes evita romper caracteres UTF-8 si un recv
                # termina justo en medio de uno de ellos.
                buffer += datos
                while b"\n" in buffer:
                    linea, buffer = buffer.split(b"\n", 1)
                    texto = linea.decode("utf-8", errors="replace").strip()
                    if texto:
                        add_line(ip, texto)

            except (OSError, ValueError) as exc:
                with lock_archivos:
                    print(f"[!] Error de conexión con {ip}: {exc}", flush=True)
                break

        if buffer.strip():
            with lock_archivos:
                print(f"[!] Trama incompleta sin salto de línea desde {ip}; descartada.", flush=True)


class ServidorTCP(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


if __name__ == "__main__":
    print(f"[TCP] Servidor escuchando en {HOST}:{TCP_PORT}")
    print(f"[LOGS] Un archivo por id de dispositivo, sesión {SESION}")
    print(f"[LOGS] Carpeta: {CARPETA_LOGS}")

    with ServidorTCP((HOST, TCP_PORT), Handler) as servidor:
        try:
            servidor.serve_forever()
        except KeyboardInterrupt:
            print("\n[!] Servidor detenido. Registros guardados.")