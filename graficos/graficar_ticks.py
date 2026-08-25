#!/usr/bin/env python3
"""
Genera DOS gráficos independientes a partir del log JSONL del ESP32:

1) ticks_en_el_tiempo.png
2) offset_acumulado_en_ms.png

Uso:
    python3 graficar_ticks_y_offset.py log_20260627_153811.txt

Para abrir ambos gráficos en pantalla:
    python3 graficar_ticks_y_offset.py log_20260627_153811.txt --show
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt


TICKS_POR_SEGUNDO = 32768.0


def cargar_registros(ruta: Path) -> list[dict]:
    registros: list[dict] = []

    with ruta.open("r", encoding="utf-8", errors="replace") as archivo:
        for numero_linea, linea in enumerate(archivo, start=1):
            linea = linea.strip()

            if not linea:
                continue

            try:
                dato = json.loads(linea)
            except json.JSONDecodeError:
                print(f"Línea {numero_linea} ignorada: JSON inválido")
                continue

            if isinstance(dato, dict) and "ms" in dato and "pps_seq" in dato:
                registros.append(dato)

    return registros


def separar_sesiones(registros: list[dict]) -> list[list[dict]]:
    """
    Separa las mediciones cuando el ESP32 se reinicia:
    - millis() disminuye; o
    - pps_seq disminuye o vuelve a comenzar.
    """
    sesiones: list[list[dict]] = []
    actual: list[dict] = []

    ms_anterior: int | None = None
    seq_anterior: int | None = None

    for dato in registros:
        ms = int(dato["ms"])
        seq = int(dato["pps_seq"])

        reinicio = (
            ms_anterior is not None
            and (
                ms < ms_anterior
                or seq <= (seq_anterior if seq_anterior is not None else seq)
            )
        )

        if reinicio and actual:
            sesiones.append(actual)
            actual = []

        actual.append(dato)
        ms_anterior = ms
        seq_anterior = seq

    if actual:
        sesiones.append(actual)

    return sesiones


def crear_grafico_ticks(sesiones: list[list[dict]], salida: Path):
    figura = plt.figure(figsize=(12, 6))

    for numero, sesion in enumerate(sesiones, start=1):
        inicio_ms = int(sesion[0]["ms"])
        tiempos: list[float] = []
        valores: list[float] = []

        for dato in sesion:
            if dato.get("first", False):
                continue

            if dato.get("interval_valid", True) is not True:
                continue

            if "ticks" not in dato:
                continue

            tiempos.append((int(dato["ms"]) - inicio_ms) / 1000.0)
            valores.append(float(dato["ticks"]))

        if tiempos:
            plt.plot(tiempos, valores, linewidth=1, label=f"Sesión {numero}")

    plt.axhline(
        TICKS_POR_SEGUNDO,
        linestyle="--",
        linewidth=1,
        label="Valor ideal: 32768",
    )

    plt.title("Ticks del RTC entre pulsos PPS")
    plt.xlabel("Tiempo desde el inicio de la sesión (s)")
    plt.ylabel("Ticks por PPS")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    figura.savefig(salida, dpi=160)

    return figura


def crear_grafico_offset(sesiones: list[list[dict]], salida: Path):
    figura = plt.figure(figsize=(12, 6))

    for numero, sesion in enumerate(sesiones, start=1):
        inicio_ms = int(sesion[0]["ms"])
        tiempos: list[float] = []
        offsets_ms: list[float] = []

        for dato in sesion:
            if "offset_ticks" not in dato:
                continue

            tiempo_s = (int(dato["ms"]) - inicio_ms) / 1000.0
            offset_ms = float(dato["offset_ticks"]) * 1000.0 / TICKS_POR_SEGUNDO

            tiempos.append(tiempo_s)
            offsets_ms.append(offset_ms)

        if tiempos:
            plt.plot(
                tiempos,
                offsets_ms,
                linewidth=1.2,
                label=f"Sesión {numero}",
            )

    plt.title("Evolución del offset acumulado")
    plt.xlabel("Tiempo desde el inicio de la sesión (s)")
    plt.ylabel("Offset acumulado (ms)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    figura.savefig(salida, dpi=160)

    return figura


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Genera los gráficos de ticks y offset acumulado."
    )
    parser.add_argument("archivo", type=Path, help="Log .txt o .jsonl")
    parser.add_argument(
        "--show",
        action="store_true",
        help="Abre los dos gráficos en ventanas.",
    )
    args = parser.parse_args()

    if not args.archivo.exists():
        raise SystemExit(f"No existe el archivo: {args.archivo}")

    registros = cargar_registros(args.archivo)

    if not registros:
        raise SystemExit("No se encontraron registros JSON válidos.")

    sesiones = separar_sesiones(registros)
    carpeta = args.archivo.parent

    archivo_ticks = carpeta / "ticks_en_el_tiempo_aging_-10.png"
    archivo_offset = carpeta / "offset_acumulado_en_ms_aging_-10.png"

    figura_ticks = crear_grafico_ticks(sesiones, archivo_ticks)
    figura_offset = crear_grafico_offset(sesiones, archivo_offset)

    print(f"Gráfico de ticks generado:  {archivo_ticks}")
    print(f"Gráfico de offset generado: {archivo_offset}")

    if args.show:
        # Se muestran ambas figuras. No se usa subplot:
        # cada gráfico permanece en una ventana independiente.
        plt.show()
    else:
        plt.close(figura_ticks)
        plt.close(figura_offset)


if __name__ == "__main__":
    main()