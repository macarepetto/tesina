#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calcular_R_phi_metodos.py

Calcula estimaciones de R_phi para el filtro de Kalman con medición:

    z = phi

a partir de un log JSONL del ESP32.

Métodos incluidos:
1) Residuo contra tendencia lineal por tramos.
2) Residuo contra media móvil / mediana móvil.
3) Valor físico por cuantización.

Uso:
    python3 calcular_R_phi_metodos.py "log_20260906_112339(2).txt"

Opcional:
    python3 calcular_R_phi_metodos.py "log.txt" --outdir salida_R_phi

"""

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


TICK_US = 1_000_000.0 / 32768.0


# Estos son los tramos que vimos en la corrida de 4 h:
# subida inicial, meseta y bajada final.
# Están en minutos desde el inicio de la corrida.
#
# Podés cambiarlos a mano si querés probar otros cortes.
TRAMOS = [
    ("subida_inicial", 0.0, 153.9),
    ("meseta", 156.6, 193.0),
    ("bajada_final", 193.0, None),  # None = hasta el final
]


# Ventanas para método 2.
# Como hay una muestra por segundo:
# 60 = 1 minuto, 300 = 5 minutos, 600 = 10 minutos.
VENTANAS_MOVILES = [60, 300, 600]


def extraer_jsons(linea: str):
    """Extrae uno o más JSON de una línea, incluso si quedaron pegados."""
    objetos = []
    depth = 0
    start = None
    in_string = False
    escape = False

    for i, ch in enumerate(linea):
        if in_string:
            if escape:
                escape = False
            elif ch == "\\":
                escape = True
            elif ch == '"':
                in_string = False
            continue

        if ch == '"':
            in_string = True
        elif ch == "{":
            if depth == 0:
                start = i
            depth += 1
        elif ch == "}":
            if depth > 0:
                depth -= 1
                if depth == 0 and start is not None:
                    chunk = linea[start:i + 1]
                    try:
                        objetos.append(json.loads(chunk))
                    except json.JSONDecodeError:
                        pass
                    start = None

    return objetos


def leer_log(path: Path) -> pd.DataFrame:
    rows = []

    with path.open("r", encoding="utf-8", errors="replace") as f:
        for line_no, line in enumerate(f, 1):
            for obj in extraer_jsons(line.strip()):
                obj["_line"] = line_no
                rows.append(obj)

    if not rows:
        raise RuntimeError("No se encontraron JSON válidos en el log.")

    df = pd.DataFrame(rows)

    for col in ["first", "interval_valid", "diff_outlier"]:
        if col not in df.columns:
            df[col] = False
        df[col] = df[col].fillna(False).astype(bool)

    for col in ["ms", "seq_delta", "offset_ticks"]:
        if col not in df.columns:
            raise RuntimeError(f"Falta la columna {col} en el log.")

    # Para estimar R_phi usamos solo mediciones reales de offset:
    # - no primera muestra
    # - intervalo válido
    # - seq_delta = 1
    # - sin outliers
    data = df[
        (df["first"] == False) &
        (df["interval_valid"] == True) &
        (df["seq_delta"] == 1) &
        (df["diff_outlier"] == False)
    ].copy()

    if data.empty:
        raise RuntimeError("No quedaron muestras válidas para calcular R_phi.")

    data = data.sort_values("ms").reset_index(drop=True)
    data["t_min"] = (data["ms"].astype(float) - data["ms"].astype(float).iloc[0]) / 60000.0
    data["t_h"] = data["t_min"] / 60.0
    data["offset_ticks"] = data["offset_ticks"].astype(float)

    return data


def resumen_residuo(nombre: str, residuo: np.ndarray) -> dict:
    residuo = residuo[np.isfinite(residuo)]

    sigma_muestral = float(np.std(residuo, ddof=1))
    r_muestral = float(np.var(residuo, ddof=1))

    return {
        "metodo": nombre,
        "muestras": int(len(residuo)),
        "media_residuo_ticks": float(np.mean(residuo)),
        "sigma_ticks": sigma_muestral,
        "R_phi_ticks2": r_muestral,
        "sigma_us": sigma_muestral * TICK_US,
        "R_phi_us2": r_muestral * TICK_US * TICK_US,
        "residuo_abs_medio_ticks": float(np.mean(np.abs(residuo))),
        "residuo_abs_p95_ticks": float(np.percentile(np.abs(residuo), 95)),
        "residuo_abs_max_ticks": float(np.max(np.abs(residuo))),
    }


def metodo_1_lineal_por_tramos(data: pd.DataFrame):
    """
    Ajusta una recta en cada tramo y calcula:
        residuo = offset_ticks - recta_del_tramo
    """
    partes = []
    resumen_tramos = []

    for nombre, inicio_min, fin_min in TRAMOS:
        if fin_min is None:
            mask = data["t_min"] >= inicio_min
        else:
            mask = (data["t_min"] >= inicio_min) & (data["t_min"] < fin_min)

        part = data[mask].copy()
        if len(part) < 2:
            continue

        m, b = np.polyfit(part["t_h"], part["offset_ticks"], 1)
        tendencia = m * part["t_h"].to_numpy(float) + b
        residuo = part["offset_ticks"].to_numpy(float) - tendencia

        part["tendencia_lineal_tramo"] = tendencia
        part["residuo_lineal_tramo"] = residuo
        part["tramo"] = nombre
        partes.append(part)

        res = resumen_residuo(f"lineal_por_tramos_{nombre}", residuo)
        res["pendiente_tramo_ticks_h"] = float(m)
        res["inicio_min"] = inicio_min
        res["fin_min"] = float(part["t_min"].iloc[-1]) if fin_min is None else fin_min
        resumen_tramos.append(res)

    combinado = pd.concat(partes, ignore_index=True)
    resumen_global = resumen_residuo(
        "lineal_por_tramos_GLOBAL",
        combinado["residuo_lineal_tramo"].to_numpy(float)
    )

    return combinado, pd.DataFrame([resumen_global] + resumen_tramos)


def metodo_2_media_mediana_movil(data: pd.DataFrame):
    """
    Calcula una tendencia lenta con media móvil y mediana móvil.
    Luego:
        residuo = offset_ticks - tendencia_movil
    """
    resultados = []
    resumenes = []

    s = data["offset_ticks"]

    for ventana in VENTANAS_MOVILES:
        # center=True: compara cada punto con una tendencia local centrada.
        # min_periods: evita perder demasiados puntos en los bordes.
        media = s.rolling(window=ventana, center=True, min_periods=max(10, ventana // 3)).mean()
        mediana = s.rolling(window=ventana, center=True, min_periods=max(10, ventana // 3)).median()

        for tipo, tendencia in [("media_movil", media), ("mediana_movil", mediana)]:
            temp = data.copy()
            temp["ventana_muestras"] = ventana
            temp["ventana_min_aprox"] = ventana / 60.0
            temp["tipo"] = tipo
            temp["tendencia_movil"] = tendencia
            temp["residuo_movil"] = temp["offset_ticks"] - temp["tendencia_movil"]

            temp_valid = temp.dropna(subset=["residuo_movil"]).copy()
            resultados.append(temp_valid)

            resumen = resumen_residuo(
                f"{tipo}_{ventana}_muestras",
                temp_valid["residuo_movil"].to_numpy(float)
            )
            resumen["ventana_muestras"] = ventana
            resumen["ventana_min_aprox"] = ventana / 60.0
            resumenes.append(resumen)

    return pd.concat(resultados, ignore_index=True), pd.DataFrame(resumenes)


def metodo_3_cuantizacion():
    """
    Estimación física simple por cuantización.

    Si la medición tiene resolución de 1 tick, el error de cuantización
    ideal uniformemente distribuido entre -0.5 y +0.5 tick tiene:

        sigma = paso / sqrt(12) = 1/sqrt(12) = 0.2887 ticks
        R = sigma^2 = 1/12 = 0.0833 ticks^2

    Una aproximación conservadora simple es asumir:
        sigma ≈ 0.5 tick
        R ≈ 0.25 ticks^2
    """
    sigma_ideal = 1.0 / np.sqrt(12.0)
    r_ideal = sigma_ideal ** 2

    sigma_conservador = 0.5
    r_conservador = sigma_conservador ** 2

    return pd.DataFrame([
        {
            "metodo": "cuantizacion_ideal_uniforme_±0.5_tick",
            "muestras": None,
            "media_residuo_ticks": 0.0,
            "sigma_ticks": sigma_ideal,
            "R_phi_ticks2": r_ideal,
            "sigma_us": sigma_ideal * TICK_US,
            "R_phi_us2": r_ideal * TICK_US * TICK_US,
            "residuo_abs_medio_ticks": None,
            "residuo_abs_p95_ticks": None,
            "residuo_abs_max_ticks": None,
        },
        {
            "metodo": "cuantizacion_conservadora_sigma_0p5_tick",
            "muestras": None,
            "media_residuo_ticks": 0.0,
            "sigma_ticks": sigma_conservador,
            "R_phi_ticks2": r_conservador,
            "sigma_us": sigma_conservador * TICK_US,
            "R_phi_us2": r_conservador * TICK_US * TICK_US,
            "residuo_abs_medio_ticks": None,
            "residuo_abs_p95_ticks": None,
            "residuo_abs_max_ticks": None,
        }
    ])


def graficar(data, lineal, moviles, resumen, outdir: Path):
    # Método 1
    plt.figure(figsize=(12, 7))
    plt.plot(data["t_min"], data["offset_ticks"], label="offset_ticks crudo", linewidth=1.0)

    for tramo in lineal["tramo"].unique():
        part = lineal[lineal["tramo"] == tramo]
        plt.plot(part["t_min"], part["tendencia_lineal_tramo"],
                 linestyle="--", label=f"tendencia lineal: {tramo}")

    plt.xlabel("Tiempo transcurrido (minutos)")
    plt.ylabel("Offset (ticks)")
    plt.title("Método 1: tendencia lineal por tramos")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / "metodo1_lineal_por_tramos.png", dpi=200)
    plt.close()

    # Residuo método 1
    plt.figure(figsize=(12, 5))
    plt.plot(lineal["t_min"], lineal["residuo_lineal_tramo"], linewidth=1.0)
    plt.axhline(0, linestyle="--")
    plt.xlabel("Tiempo transcurrido (minutos)")
    plt.ylabel("Residuo (ticks)")
    plt.title("Método 1: residuo respecto de la tendencia lineal por tramos")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(outdir / "metodo1_residuo_lineal_por_tramos.png", dpi=200)
    plt.close()

    # Método 2: un gráfico por ventana/tipo
    for (tipo, ventana), part in moviles.groupby(["tipo", "ventana_muestras"]):
        plt.figure(figsize=(12, 7))
        plt.plot(part["t_min"], part["offset_ticks"], label="offset_ticks crudo", linewidth=1.0)
        plt.plot(part["t_min"], part["tendencia_movil"],
                 label=f"{tipo}, ventana={ventana} muestras")
        plt.xlabel("Tiempo transcurrido (minutos)")
        plt.ylabel("Offset (ticks)")
        plt.title(f"Método 2: {tipo}, ventana {ventana} muestras")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig(outdir / f"metodo2_{tipo}_{ventana}_muestras.png", dpi=200)
        plt.close()

        plt.figure(figsize=(12, 5))
        plt.plot(part["t_min"], part["residuo_movil"], linewidth=1.0)
        plt.axhline(0, linestyle="--")
        plt.xlabel("Tiempo transcurrido (minutos)")
        plt.ylabel("Residuo (ticks)")
        plt.title(f"Método 2: residuo {tipo}, ventana {ventana} muestras")
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(outdir / f"metodo2_residuo_{tipo}_{ventana}_muestras.png", dpi=200)
        plt.close()

    # Comparación de R
    resumen_plot = resumen.dropna(subset=["R_phi_ticks2"]).copy()
    resumen_plot = resumen_plot.sort_values("R_phi_ticks2")

    plt.figure(figsize=(11, 7))
    plt.barh(resumen_plot["metodo"], resumen_plot["R_phi_ticks2"])
    plt.xlabel("R_phi estimado (ticks²)")
    plt.title("Comparación de estimaciones de R_phi")
    plt.grid(True, axis="x", alpha=0.3)
    plt.tight_layout()
    plt.savefig(outdir / "comparacion_R_phi_metodos.png", dpi=200)
    plt.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log", help="Archivo de log JSONL del ESP32")
    parser.add_argument("--outdir", default="salida_R_phi",
                        help="Carpeta de salida")
    args = parser.parse_args()

    log_path = Path(args.log)
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    data = leer_log(log_path)

    lineal, resumen_lineal = metodo_1_lineal_por_tramos(data)
    moviles, resumen_moviles = metodo_2_media_mediana_movil(data)
    resumen_cuantizacion = metodo_3_cuantizacion()

    resumen = pd.concat(
        [resumen_lineal, resumen_moviles, resumen_cuantizacion],
        ignore_index=True
    )

    data.to_csv(outdir / "muestras_validas_offset.csv", index=False)
    lineal.to_csv(outdir / "metodo1_residuos_lineal_por_tramos.csv", index=False)
    moviles.to_csv(outdir / "metodo2_residuos_moviles.csv", index=False)
    resumen.to_csv(outdir / "resumen_R_phi_metodos.csv", index=False)

    graficar(data, lineal, moviles, resumen, outdir)

    print("Resumen de R_phi estimado:")
    columnas = ["metodo", "muestras", "sigma_ticks", "R_phi_ticks2", "sigma_us"]
    print(resumen[columnas].to_string(index=False))

    print()
    print(f"Archivos generados en: {outdir.resolve()}")
    print(" - resumen_R_phi_metodos.csv")
    print(" - metodo1_lineal_por_tramos.png")
    print(" - metodo1_residuo_lineal_por_tramos.png")
    print(" - metodo2_<tipo>_<ventana>_muestras.png")
    print(" - metodo2_residuo_<tipo>_<ventana>_muestras.png")
    print(" - comparacion_R_phi_metodos.png")


if __name__ == "__main__":
    main()
