#!/usr/bin/env python3
"""
Grafica los ticks y el offset acumulado, y calcula una regresión lineal.

Uso:
    python3 graficar_aging_con_regresion.py log_aging_0.txt

Opcional:
    python3 graficar_aging_con_regresion.py log_aging_0.txt --show

Genera:
    - ticks_aging.png
    - offset_aging_con_regresion.png
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt


TICKS_IDEALES = 32768.0


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


def regresion_lineal(x: list[float], y: list[float]) -> tuple[float, float, float]:
    """
    Devuelve:
      pendiente, ordenada al origen y R²
    """
    if len(x) != len(y) or len(x) < 2:
        raise ValueError("Se necesitan al menos dos puntos.")

    n = float(len(x))
    suma_x = sum(x)
    suma_y = sum(y)
    suma_xx = sum(valor * valor for valor in x)
    suma_xy = sum(a * b for a, b in zip(x, y))

    denominador = n * suma_xx - suma_x * suma_x

    if denominador == 0:
        raise ValueError("No se puede calcular la regresión.")

    pendiente = (
        n * suma_xy - suma_x * suma_y
    ) / denominador

    intercepto = (
        suma_y - pendiente * suma_x
    ) / n

    predichos = [
        pendiente * valor + intercepto
        for valor in x
    ]

    promedio_y = suma_y / n

    ss_res = sum(
        (real - predicho) ** 2
        for real, predicho in zip(y, predichos)
    )

    ss_tot = sum(
        (real - promedio_y) ** 2
        for real in y
    )

    r_cuadrado = (
        1.0 - ss_res / ss_tot
        if ss_tot != 0
        else 1.0
    )

    return pendiente, intercepto, r_cuadrado


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Grafica una prueba de aging y calcula su regresión lineal."
    )
    parser.add_argument("archivo", type=Path, help="Log JSONL del ESP32")
    parser.add_argument(
        "--show",
        action="store_true",
        help="Abre los gráficos además de guardar los PNG.",
    )
    args = parser.parse_args()

    if not args.archivo.exists():
        raise SystemExit(f"No existe el archivo: {args.archivo}")

    registros = cargar_registros(args.archivo)

    if not registros:
        raise SystemExit("No se encontraron registros JSON válidos.")

    sesiones = separar_sesiones(registros)

    # Se usa la sesión más larga para evitar tomar un tramo corto previo
    # a un reinicio por cambio de USB a powerbank.
    sesion = max(sesiones, key=len)

    inicio_ms = int(sesion[0]["ms"])

    tiempos_h_offset: list[float] = []
    offsets_ms: list[float] = []

    tiempos_s_ticks: list[float] = []
    ticks: list[float] = []

    for dato in sesion:
        tiempo_s = (
            int(dato["ms"]) - inicio_ms
        ) / 1000.0

        if "offset_ticks" in dato:
            tiempos_h_offset.append(tiempo_s / 3600.0)
            offsets_ms.append(
                float(dato["offset_ticks"])
                * 1000.0
                / TICKS_IDEALES
            )

        if (
            not dato.get("first", False)
            and dato.get("interval_valid", True) is True
            and "ticks" in dato
        ):
            tiempos_s_ticks.append(tiempo_s)
            ticks.append(float(dato["ticks"]))

    if len(tiempos_h_offset) < 2:
        raise SystemExit("No hay suficientes puntos de offset.")

    pendiente, intercepto, r2 = regresion_lineal(
        tiempos_h_offset,
        offsets_ms,
    )

    ppm = pendiente / 3.6
    ms_dia = pendiente * 24.0

    # ---------------- Gráfico de ticks ----------------
    figura_ticks = plt.figure(figsize=(12, 6))

    plt.plot(
        tiempos_s_ticks,
        ticks,
        linewidth=1,
        label="Ticks medidos",
    )

    plt.axhline(
        TICKS_IDEALES,
        linestyle="--",
        linewidth=1,
        label="Valor ideal: 32768",
    )

    plt.ticklabel_format(
        axis="y",
        style="plain",
        useOffset=False,
    )

    plt.title("Ticks del RTC entre pulsos PPS")
    plt.xlabel("Tiempo desde el inicio de la sesión (s)")
    plt.ylabel("Ticks por PPS")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    salida_ticks = args.archivo.parent / "ticks_aging_-14.png"
    figura_ticks.savefig(salida_ticks, dpi=160)

    # ------------- Gráfico offset + regresión -------------
    figura_offset = plt.figure(figsize=(12, 6))

    plt.plot(
        tiempos_h_offset,
        offsets_ms,
        linewidth=1.2,
        label="Offset acumulado medido",
    )

    valores_regresion = [
        pendiente * tiempo + intercepto
        for tiempo in tiempos_h_offset
    ]

    plt.plot(
        tiempos_h_offset,
        valores_regresion,
        linestyle="--",
        linewidth=1.5,
        label=(
            f"Regresión: y = {pendiente:.4f}x "
            f"+ {intercepto:.4f} | R²={r2:.6f}"
        ),
    )

    plt.title("Offset acumulado y regresión lineal")
    plt.xlabel("Tiempo desde el inicio de la sesión (h)")
    plt.ylabel("Offset acumulado (ms)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    salida_offset = (
        args.archivo.parent
        / "offset_aging_con_regresion_-14.png"
    )
    figura_offset.savefig(salida_offset, dpi=160)

    print()
    print("RESULTADOS DE LA REGRESIÓN")
    print(f"Sesiones detectadas: {len(sesiones)}")
    print(f"Sesión analizada: la más larga ({len(sesion)} registros)")
    print(f"Pendiente:          {pendiente:.6f} ms/h")
    print(f"Intercepto:         {intercepto:.6f} ms")
    print(f"R²:                 {r2:.9f}")
    print(f"Frecuencia relativa:{ppm:.6f} ppm")
    print(f"Proyección diaria:  {ms_dia:.6f} ms/día")
    print()
    print(f"Gráfico de ticks:   {salida_ticks}")
    print(f"Gráfico de offset:  {salida_offset}")

    if args.show:
        plt.show()
    else:
        plt.close(figura_ticks)
        plt.close(figura_offset)


if __name__ == "__main__":
    main()
