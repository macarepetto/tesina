#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
probar_matrices_kalman.py

Reprocesa offline un log del ESP32 y compara distintas matrices de ruido
para el filtro de Kalman con medición escalar:

    z = phi
    H = [1 0]

Uso:
    python3 probar_matrices_kalman.py "log_20260906_112339.txt"

Salida:
    - comparacion_matrices_kalman.png
    - comparacion_residuales_kalman.png
    - comparacion_deriva_f_kalman.png
    - pendiente_<configuracion>.png
    - metricas_matrices_kalman.csv

Dependencias:
    pip install numpy pandas matplotlib
"""

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ============================================================
# CONFIGURACIONES A PROBAR
# ============================================================
#
# q_phi: ruido de proceso del offset
# q_f:   ruido de proceso de la deriva
# r_phi: ruido de medición del offset z = phi
#
# Para que el Kalman "promedie más":
#   - aumentar r_phi
#   - bajar q_phi
#   - bajar q_f
#
# Para que responda más rápido:
#   - bajar r_phi
#   - subir q_phi
#   - subir q_f
#
CONFIGS = [
    {
        "nombre": "actual_R1_Q1e-4_1e-7",
        "etiqueta": "Actual: R=1, Q=(1e-4, 1e-7)",
        "q_phi": 1e-4,
        "q_f": 1e-7,
        "r_phi": 1.0,
        "p_phi_0": 100.0,
        "p_f_0": 1.0,
    },
    {
        "nombre": "calmo_R1_Q1e-6_1e-10",
        "etiqueta": "Mas calmo: R=1, Q=(1e-6, 1e-10)",
        "q_phi": 1e-6,
        "q_f": 1e-10,
        "r_phi": 1.0,
        "p_phi_0": 100.0,
        "p_f_0": 1.0,
    },
    {
        "nombre": "calmo_R1_Q1e-6_1e-11",
        "etiqueta": "Mas calmo: R=1, Q=(1e-6, 1e-11)",
        "q_phi": 1e-6,
        "q_f": 1e-11,
        "r_phi": 1.0,
        "p_phi_0": 100.0,
        "p_f_0": 1.0,
    },
    {
        "nombre": "muy_calmo_R1_Q1e-6 1e-12",
        "etiqueta": "Muy calmo: R=1, Q=(1e-6 1e-12)",
        "q_phi": 1e-6,
        "q_f": 1e-12,
        "r_phi": 1.0,
        "p_phi_0": 100.0,
        "p_f_0": 1.0,
    },
]


TICK_US = 1_000_000.0 / 32768.0


def extraer_jsons(linea: str):
    """
    Extrae uno o más objetos JSON de una línea.
    Sirve incluso si por algún problema quedaron dos JSON pegados en la misma línea.
    """
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


def leer_log_jsonl(path: Path) -> pd.DataFrame:
    rows = []

    with path.open("r", encoding="utf-8", errors="replace") as f:
        for line_no, line in enumerate(f, 1):
            for obj in extraer_jsons(line.strip()):
                obj["_line"] = line_no
                rows.append(obj)

    if not rows:
        raise RuntimeError(f"No pude leer objetos JSON válidos en {path}")

    df = pd.DataFrame(rows)

    # Normalización básica de columnas esperadas
    for col in ["first", "interval_valid", "diff_outlier", "kalman_actualizado"]:
        if col not in df.columns:
            df[col] = False
        df[col] = df[col].fillna(False).astype(bool)

    if "offset_medido_disponible" not in df.columns:
        # Los logs viejos no tenían este campo; asumimos que si no está, hay medición.
        df["offset_medido_disponible"] = True
    df["offset_medido_disponible"] = df["offset_medido_disponible"].fillna(True).astype(bool)

    for col in ["ms", "pps_seq", "seq_delta", "offset_ticks"]:
        if col not in df.columns:
            raise RuntimeError(f"El log no tiene la columna esperada: {col}")

    df = df.sort_values(["ms", "pps_seq"]).reset_index(drop=True)
    return df


def preparar_muestras(df: pd.DataFrame) -> pd.DataFrame:
    """
    Para comparar matrices usamos las filas donde existe offset medido.
    Las filas HOLDOVER o REENGANCHE se pueden conservar para graficar contexto,
    pero no se usan como medición del filtro.
    """
    data = df[df["first"] == False].copy()

    modo = data["modo"] if "modo" in data.columns else ""

    data["usar_como_medicion"] = (
        (data["interval_valid"] == True) &
        (data["diff_outlier"] == False) &
        (data["offset_medido_disponible"] == True) &
        (data["seq_delta"].fillna(0).astype(int) == 1) &
        (modo != "HOLDOVER") &
        (modo != "PPS_REENGANCHE")
    )

    # Para un análisis de la corrida normal, conviene quedarse con filas que tengan offset.
    data = data.dropna(subset=["offset_ticks", "ms"]).copy()

    data["t_h"] = (data["ms"].astype(float) - data["ms"].astype(float).iloc[0]) / 3_600_000.0
    data["t_min"] = data["t_h"] * 60.0

    return data.reset_index(drop=True)


def ejecutar_kalman_zphi(data: pd.DataFrame, cfg: dict) -> pd.DataFrame:
    """
    Reproduce offline el filtro:

        x = [phi, f]^T

        F = [1 1
             0 1]

        z = phi
        H = [1 0]

    En filas no utilizables como medición, conserva el último estado.
    Para el log normal de 4 h deberían usarse prácticamente todas las filas.
    """

    phi = 0.0
    f = 0.0

    p00 = float(cfg["p_phi_0"])
    p01 = 0.0
    p10 = 0.0
    p11 = float(cfg["p_f_0"])

    q_phi = float(cfg["q_phi"])
    q_f = float(cfg["q_f"])
    r_phi = float(cfg["r_phi"])

    out = []

    for _, row in data.iterrows():
        z_phi = float(row["offset_ticks"])

        actualizado = False
        predicho = False

        if bool(row["usar_como_medicion"]):
            # --------------------
            # PREDICCIÓN
            # --------------------
            phi_pred = phi + f
            f_pred = f

            p00_pred = p00 + p01 + p10 + p11 + q_phi
            p01_pred = p01 + p11
            p10_pred = p10 + p11
            p11_pred = p11 + q_f

            phi = phi_pred
            f = f_pred
            p00, p01, p10, p11 = p00_pred, p01_pred, p10_pred, p11_pred
            predicho = True

            # --------------------
            # ACTUALIZACIÓN z = phi
            # --------------------
            y = z_phi - phi
            s = p00 + r_phi

            k_phi = p00 / s
            k_f = p10 / s

            old_p00, old_p01, old_p10, old_p11 = p00, p01, p10, p11

            phi = phi + k_phi * y
            f = f + k_f * y

            # P = (I - K H) P
            p00 = (1.0 - k_phi) * old_p00
            p01 = (1.0 - k_phi) * old_p01
            p10 = old_p10 - k_f * old_p00
            p11 = old_p11 - k_f * old_p01

            actualizado = True

        residual_us = (z_phi - phi) * TICK_US

        out.append({
            "t_h": float(row["t_h"]),
            "t_min": float(row["t_min"]),
            "pps_seq": int(row["pps_seq"]),
            "offset_ticks": z_phi,
            "offset_us": z_phi * TICK_US,
            "phi": phi,
            "phi_us": phi * TICK_US,
            "f": f,
            "p00": p00,
            "p11": p11,
            "residual_us": residual_us,
            "predicho": predicho,
            "actualizado": actualizado,
            "modo_original": row.get("modo", ""),
        })

    return pd.DataFrame(out)


def ajuste_lineal(x, y):
    if len(x) < 2:
        return np.nan, np.nan, np.full_like(x, np.nan, dtype=float), np.nan

    coef = np.polyfit(x, y, 1)
    fit = np.poly1d(coef)(x)

    ss_res = float(np.sum((y - fit) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot else np.nan

    return float(coef[0]), float(coef[1]), fit, r2


def ticks_h_a_ppm(slope_ticks_h):
    return slope_ticks_h / (32768.0 * 3600.0) * 1e6


def ticks_h_a_ms_dia(slope_ticks_h):
    return slope_ticks_h * 24.0 / 32768.0 * 1000.0


def generar_graficos_y_metricas(data: pd.DataFrame, resultados: dict, outdir: Path):
    outdir.mkdir(parents=True, exist_ok=True)

    metricas = []

    t_h = data["t_h"].to_numpy(float)
    offset = data["offset_ticks"].to_numpy(float)

    slope_offset, _, fit_offset, r2_offset = ajuste_lineal(t_h, offset)

    smooth_offset = float(np.mean(np.abs(np.diff(offset)))) if len(offset) > 1 else np.nan

    # ============================================================
    # Gráfico 1: comparación principal
    # ============================================================
    plt.figure(figsize=(12, 7))
    plt.plot(t_h, offset, label="offset crudo", linewidth=1.0)

    for cfg_name, res in resultados.items():
        plt.plot(
            res["t_h"],
            res["phi"],
            label=res.attrs["etiqueta"],
            linewidth=1.2
        )

    plt.xlabel("Tiempo transcurrido (horas)")
    plt.ylabel("Offset / phi (ticks)")
    plt.title("Comparación offline de matrices de ruido - Kalman z = phi")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / "comparacion_matrices_kalman.png", dpi=200)
    plt.close()

    # ============================================================
    # Gráfico 2: residual
    # ============================================================
    plt.figure(figsize=(12, 6))

    for cfg_name, res in resultados.items():
        plt.plot(
            res["t_h"],
            res["residual_us"],
            label=res.attrs["etiqueta"],
            linewidth=1.2
        )

    plt.axhline(0, linestyle="--")
    plt.xlabel("Tiempo transcurrido (horas)")
    plt.ylabel("Residual (µs) = offset_us - phi_us")
    plt.title("Residual para distintas matrices de ruido")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / "comparacion_residuales_kalman.png", dpi=200)
    plt.close()

    # ============================================================
    # Gráfico 3: deriva estimada f
    # ============================================================
    plt.figure(figsize=(12, 6))

    for cfg_name, res in resultados.items():
        plt.plot(
            res["t_h"],
            res["f"],
            label=res.attrs["etiqueta"],
            linewidth=1.2
        )

    plt.axhline(0, linestyle="--")
    plt.xlabel("Tiempo transcurrido (horas)")
    plt.ylabel("f estimada (ticks/PPS)")
    plt.title("Deriva estimada para distintas matrices de ruido")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / "comparacion_deriva_f_kalman.png", dpi=200)
    plt.close()

    # ============================================================
    # Gráfico 4: offset crudo + tendencias para cada configuración
    # ============================================================
    for cfg_name, res in resultados.items():
        phi = res["phi"].to_numpy(float)
        slope_phi, _, fit_phi, r2_phi = ajuste_lineal(t_h, phi)

        plt.figure(figsize=(12, 7))
        plt.plot(t_h, offset, label="offset crudo", linewidth=1.0)
        plt.plot(t_h, phi, label=res.attrs["etiqueta"], linewidth=1.3)
        plt.plot(t_h, fit_offset, linestyle="--",
                 label=f"tendencia offset ({slope_offset:.2f} ticks/h)")
        plt.plot(t_h, fit_phi, linestyle="--",
                 label=f"tendencia Kalman ({slope_phi:.2f} ticks/h)")

        plt.xlabel("Tiempo transcurrido (horas)")
        plt.ylabel("Ticks")
        plt.title(f"Offset crudo vs Kalman - {cfg_name}")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig(outdir / f"pendiente_{cfg_name}.png", dpi=200)
        plt.close()

    # ============================================================
    # Métricas
    # ============================================================
    for cfg_name, res in resultados.items():
        phi = res["phi"].to_numpy(float)
        residual = res["residual_us"].to_numpy(float)

        slope_phi, _, _, r2_phi = ajuste_lineal(t_h, phi)

        smooth_phi = float(np.mean(np.abs(np.diff(phi)))) if len(phi) > 1 else np.nan

        metricas.append({
            "configuracion": cfg_name,
            "etiqueta": res.attrs["etiqueta"],
            "q_phi": res.attrs["q_phi"],
            "q_f": res.attrs["q_f"],
            "r_phi": res.attrs["r_phi"],
            "p_phi_0": res.attrs["p_phi_0"],
            "p_f_0": res.attrs["p_f_0"],
            "muestras": len(res),
            "offset_suavidad_ticks_paso": smooth_offset,
            "kalman_suavidad_ticks_paso": smooth_phi,
            "factor_suavizado_offset_sobre_kalman": smooth_offset / smooth_phi if smooth_phi else np.nan,
            "residual_abs_medio_us": float(np.mean(np.abs(residual))),
            "residual_abs_p95_us": float(np.percentile(np.abs(residual), 95)),
            "residual_abs_max_us": float(np.max(np.abs(residual))),
            "phi_final_ticks": float(phi[-1]),
            "phi_final_us": float(phi[-1] * TICK_US),
            "f_final_ticks_pps": float(res["f"].iloc[-1]),
            "pendiente_offset_ticks_h": slope_offset,
            "pendiente_offset_ppm": ticks_h_a_ppm(slope_offset),
            "pendiente_offset_ms_dia": ticks_h_a_ms_dia(slope_offset),
            "r2_offset": r2_offset,
            "pendiente_kalman_ticks_h": slope_phi,
            "pendiente_kalman_ppm": ticks_h_a_ppm(slope_phi),
            "pendiente_kalman_ms_dia": ticks_h_a_ms_dia(slope_phi),
            "r2_kalman": r2_phi,
        })

    metricas_df = pd.DataFrame(metricas)
    metricas_df.to_csv(outdir / "metricas_matrices_kalman.csv", index=False)

    return metricas_df


def main():
    parser = argparse.ArgumentParser(
        description="Compara matrices de ruido para Kalman z=phi usando un log JSONL del ESP32."
    )
    parser.add_argument(
        "log",
        type=str,
        help="Ruta al log. Ejemplo: 'log_20260906_112339(2).txt'"
    )
    parser.add_argument(
        "--outdir",
        type=str,
        default="salida_kalman_matrices",
        help="Carpeta de salida. Por defecto: salida_kalman_matrices"
    )

    args = parser.parse_args()

    log_path = Path(args.log)
    outdir = Path(args.outdir)

    df_raw = leer_log_jsonl(log_path)
    data = preparar_muestras(df_raw)

    if data.empty:
        raise RuntimeError("No quedaron muestras útiles para analizar.")

    print(f"Archivo: {log_path}")
    print(f"JSON leídos: {len(df_raw)}")
    print(f"Muestras analizadas: {len(data)}")
    print(f"Duración: {data['t_h'].iloc[-1]:.3f} h")
    print(f"Muestras usadas como medición Kalman: {int(data['usar_como_medicion'].sum())}")
    print()

    resultados = {}

    for cfg in CONFIGS:
        res = ejecutar_kalman_zphi(data, cfg)
        res.attrs.update(cfg)
        resultados[cfg["nombre"]] = res

    metricas = generar_graficos_y_metricas(data, resultados, outdir)

    print("Métricas principales:")
    cols = [
        "configuracion",
        "q_phi",
        "q_f",
        "r_phi",
        "kalman_suavidad_ticks_paso",
        "residual_abs_medio_us",
        "residual_abs_p95_us",
        "pendiente_kalman_ticks_h",
        "pendiente_kalman_ms_dia",
        "phi_final_ticks",
        "f_final_ticks_pps",
    ]
    print(metricas[cols].to_string(index=False))

    print()
    print(f"Archivos generados en: {outdir.resolve()}")
    print(" - comparacion_matrices_kalman.png")
    print(" - comparacion_residuales_kalman.png")
    print(" - comparacion_deriva_f_kalman.png")
    print(" - pendiente_<configuracion>.png")
    print(" - metricas_matrices_kalman.csv")


if __name__ == "__main__":
    main()
