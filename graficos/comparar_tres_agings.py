#!/usr/bin/env python3
"""
Superpone tres ensayos de aging en un mismo gráfico.

Ejemplo:
    python3 comparar_tres_agings.py \
        log_aging_0.txt \
        log_aging_-13.txt \
        log_aging_-14.txt \
        --labels "aging 0" "aging -13" "aging -14" \
        --show

Genera:
    comparacion_tres_agings.png

Por defecto, cada curva se normaliza para comenzar en 0 ms.
Esto facilita comparar las pendientes y la forma de cada ensayo.
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
                print(f"{ruta.name}: línea {numero_linea} ignorada")
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


def regresion_lineal(
    x: list[float],
    y: list[float],
) -> tuple[float, float, float]:
    if len(x) != len(y) or len(x) < 2:
        raise ValueError("No hay suficientes puntos para la regresión.")

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

    r2 = 1.0 - ss_res / ss_tot if ss_tot != 0 else 1.0

    return pendiente, intercepto, r2


def extraer_serie(
    ruta: Path,
    normalizar: bool,
) -> tuple[list[float], list[float]]:
    registros = cargar_registros(ruta)

    if not registros:
        raise ValueError(f"{ruta.name}: no contiene registros válidos.")

    sesiones = separar_sesiones(registros)
    sesion = max(sesiones, key=len)

    inicio_ms = int(sesion[0]["ms"])
    tiempos_h: list[float] = []
    offsets_ms: list[float] = []

    for dato in sesion:
        if "offset_ticks" not in dato:
            continue

        tiempo_h = (
            int(dato["ms"]) - inicio_ms
        ) / 3_600_000.0

        offset_ms = (
            float(dato["offset_ticks"])
            * 1000.0
            / TICKS_POR_SEGUNDO
        )

        tiempos_h.append(tiempo_h)
        offsets_ms.append(offset_ms)

    if len(tiempos_h) < 2:
        raise ValueError(f"{ruta.name}: no hay suficientes datos.")

    if normalizar:
        offset_inicial = offsets_ms[0]
        offsets_ms = [
            valor - offset_inicial
            for valor in offsets_ms
        ]

    return tiempos_h, offsets_ms


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compara tres logs de aging en un mismo gráfico."
    )

    parser.add_argument(
        "archivos",
        nargs=3,
        type=Path,
        help="Tres archivos de log JSONL.",
    )

    parser.add_argument(
        "--labels",
        nargs=3,
        metavar=("LABEL1", "LABEL2", "LABEL3"),
        help="Etiquetas para los tres ensayos.",
    )

    parser.add_argument(
        "--sin-normalizar",
        action="store_true",
        help="Mantiene el offset inicial original de cada ensayo.",
    )

    parser.add_argument(
        "--show",
        action="store_true",
        help="Abre el gráfico además de guardarlo.",
    )

    args = parser.parse_args()

    labels = (
        list(args.labels)
        if args.labels
        else [archivo.stem for archivo in args.archivos]
    )

    normalizar = not args.sin_normalizar

    figura = plt.figure(figsize=(13, 7))
    resumen: list[tuple[str, float, float, float, float]] = []

    for ruta, etiqueta in zip(args.archivos, labels):
        if not ruta.exists():
            raise SystemExit(f"No existe el archivo: {ruta}")

        tiempos_h, offsets_ms = extraer_serie(
            ruta,
            normalizar=normalizar,
        )

        pendiente, intercepto, r2 = regresion_lineal(
            tiempos_h,
            offsets_ms,
        )

        regresion = [
            pendiente * tiempo + intercepto
            for tiempo in tiempos_h
        ]

        plt.plot(
            tiempos_h,
            offsets_ms,
            linewidth=1.2,
            label=(
                f"{etiqueta}: medido "
                f"(m={pendiente:.4f} ms/h, R²={r2:.4f})"
            ),
        )

        plt.plot(
            tiempos_h,
            regresion,
            linestyle="--",
            linewidth=1.5,
            label=f"{etiqueta}: regresión",
        )

        ppm = pendiente / 3.6
        duracion_h = tiempos_h[-1] - tiempos_h[0]

        resumen.append(
            (etiqueta, pendiente, r2, ppm, duracion_h)
        )

    plt.axhline(
        0,
        linestyle=":",
        linewidth=1,
        label="Offset ideal: 0",
    )

    titulo = "Comparación del offset acumulado para tres valores de aging"

    if normalizar:
        titulo += " — curvas normalizadas al inicio"

    plt.title(titulo)
    plt.xlabel("Tiempo desde el inicio del ensayo (h)")
    plt.ylabel("Offset acumulado (ms)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    salida = Path("comparacion_tres_agings.png")
    figura.savefig(salida, dpi=170)

    print()
    print("COMPARACIÓN DE ENSAYOS")
    print(
        f"{'Ensayo':<16}"
        f"{'Pendiente (ms/h)':>20}"
        f"{'|Pendiente|':>15}"
        f"{'R²':>12}"
        f"{'ppm':>12}"
        f"{'Duración (h)':>15}"
    )

    for etiqueta, pendiente, r2, ppm, duracion_h in resumen:
        print(
            f"{etiqueta:<16}"
            f"{pendiente:>20.6f}"
            f"{abs(pendiente):>15.6f}"
            f"{r2:>12.6f}"
            f"{ppm:>12.6f}"
            f"{duracion_h:>15.3f}"
        )

    mejor = min(resumen, key=lambda fila: abs(fila[1]))

    print()
    print(
        "Menor pendiente absoluta: "
        f"{mejor[0]} ({mejor[1]:.6f} ms/h)"
    )
    print(f"Gráfico guardado en: {salida.resolve()}")

    if args.show:
        plt.show()
    else:
        plt.close(figura)


if __name__ == "__main__":
    main()
