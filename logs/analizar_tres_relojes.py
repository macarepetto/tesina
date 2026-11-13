#!/usr/bin/env python3
"""Muestra GNSS/PPS, Kalman y RTC/SQW por prototipo (solo lectura)."""
import argparse
import json
import math
from pathlib import Path
import re
from datetime import datetime
from collections import defaultdict, Counter
from statistics import fmean
import zipfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def registros(archivo):
    """Lee JSONL dentro de TXT o ZIP, sin descomprimir el ZIP."""
    ruta = Path(archivo)
    if ruta.suffix.lower() == ".zip":
        with zipfile.ZipFile(ruta) as z:
            for nombre in z.namelist():
                if not nombre.lower().endswith((".txt", ".jsonl", ".log")):
                    continue
                with z.open(nombre) as f:
                    for linea in f:
                        try:
                            dato = json.loads(linea)
                            if isinstance(dato, dict):
                                yield nombre, dato
                        except (ValueError, UnicodeDecodeError):
                            continue
    else:
        with ruta.open(encoding="utf-8-sig", errors="replace") as f:
            for linea in f:
                try:
                    dato = json.loads(linea)
                    if isinstance(dato, dict):
                        yield ruta.name, dato
                except ValueError:
                    continue


def instante(texto):
    if not isinstance(texto, str) or not texto:
        raise ValueError("Marca temporal vacía")
    return datetime.fromisoformat(texto.replace("Z", "+00:00"))


def microsegundos(a, b):
    return (a - b).total_seconds() * 1_000_000


def duracion(seg):
    seg = max(0, int(round(seg)))
    h, rem = divmod(seg, 3600)
    m, s = divmod(rem, 60)
    return f"{h} h {m} min {s} s"


def p95_abs(datos):
    valores = sorted(abs(v) for v in datos)
    return valores[max(0, math.ceil(0.95 * len(valores)) - 1)]


def hora_sqw(registro):
    # Nombre actual; el segundo aparece solo en firmware anteriores.
    return registro.get("reloj_rtc_calendario_sqw") or registro.get("reloj_rtc_sqw")


def relojes_comparables(registro):
    """Extrae los errores frente a GNSS solo si hay tres horas simultáneas."""
    try:
        gnss = instante(registro["reloj_gnss_pps"])
        kalman = instante(registro["reloj_kalman"])
        sqw = instante(hora_sqw(registro))
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
    error_k = microsegundos(kalman, gnss)
    error_s = microsegundos(sqw, gnss)
    fino = registro.get("desfase_kalman_us")
    if isinstance(fino, (float, int)) and math.isfinite(fino):
        if abs(float(fino) - error_k) <= 1.1:
            error_k = float(fino)
    return gnss, kalman, sqw, error_k, error_s


def detalle_registro(etiqueta, registro):
    if registro is None:
        print(f"  {etiqueta}: sin registro disponible")
        return
    comparable = relojes_comparables(registro)
    ms = registro.get("ms", "?")
    seq = registro.get("pps_seq", "?")
    print(f"  {etiqueta}: ms={ms}, PPS={seq}, modo={registro.get('modo')}")
    if comparable is not None:
        print(f"    GNSS/PPS: {comparable[0].isoformat(sep=' ')}")
        print(f"    Kalman:   {comparable[1].isoformat(sep=' ')} "
              f"(desfase {comparable[3]:+.3f} µs)")
        print(f"    RTC/SQW:  {comparable[2].isoformat(sep=' ')} "
              f"(desfase {comparable[4]:+.3f} µs)")
    else:
        print("    No se puede evaluar error absoluto: falta GNSS/PPS "
              "o alguno de los tres relojes.")
        if registro.get("reloj_kalman"):
            print(f"    Kalman publicado: {registro['reloj_kalman']}")
        if hora_sqw(registro):
            print(f"    RTC/SQW publicado: {hora_sqw(registro)}")
    print(f"    offset_ticks={registro.get('offset_ticks')}, "
          f"phi_us={registro.get('kalman_phi_us')}, "
          f"kalman_actualizado={registro.get('kalman_actualizado')}, "
          f"relojes_estado={registro.get('relojes_estado')}")


def informar_holdover_proto1(registros):
    """Audita y analiza la recuperación de PPS sin modificar el Kalman."""
    print("\n" + "=" * 65)
    print("ANÁLISIS ESPECÍFICO DE HOLDOVER Y REENGANCHE — PROTOTIPO 1")
    print("=" * 65)

    episodios = []
    primero = ultimo = None
    for indice, registro in enumerate(registros):
        modo = registro.get("modo", "")
        if modo == "HOLDOVER":
            if primero is None:
                primero = indice
            ultimo = indice
        elif primero is not None and modo.startswith("PPS_"):
            episodios.append((primero, ultimo, indice))
            primero = ultimo = None
    if primero is not None:
        episodios.append((primero, ultimo, None))

    if not episodios:
        print("No hay registros con modo HOLDOVER en el log del prototipo 1.")
        print("Por lo tanto, esta corrida NO permite evaluar una recuperación real.")
        return

    for numero, (inicio, fin, vuelta) in enumerate(episodios, start=1):
        durante = registros[inicio:fin + 1]
        anterior = next((r for r in reversed(registros[:inicio])
                         if relojes_comparables(r) is not None), None)
        regreso = registros[vuelta] if vuelta is not None else None
        limite = next((i for i in range((vuelta or fin) + 1, len(registros))
                       if registros[i].get("modo") == "HOLDOVER"), len(registros))
        despues = registros[(vuelta + 1) if vuelta is not None else fin + 1:limite]
        primera_actualizacion = next((r for r in despues
                                     if r.get("kalman_actualizado") is True), None)
        primer_comparable = next((r for r in despues
                                 if relojes_comparables(r) is not None), None)

        print(f"\n--- Episodio {numero} ---")
        print(f"Predicciones registradas durante holdover: {len(durante)}")
        if anterior is not None and regreso is not None:
            dur_ms = regreso.get("ms", 0) - anterior.get("ms", 0)
            print(f"Tiempo desde último registro comparable hasta regreso: "
                  f"{duracion(dur_ms / 1000)}")
        pasos = durante[-1].get("holdover_steps", durante[-1].get("holdover_pasos"))
        print(f"Último holdover_steps: {pasos}")
        indebidas = [r for r in durante if r.get("kalman_actualizado") is True]
        predichas = sum(r.get("kalman_prediccion", r.get("kalman_predicho")) is True
                        for r in durante)
        print(f"Durante HOLDOVER: predict=true en {predichas} registros; "
              f"update=true en {len(indebidas)} registros")
        if indebidas:
            print("  ¡ALERTA! Hay actualizaciones Kalman durante HOLDOVER.")
        else:
            print("  OK: el log no registra actualizaciones con mediciones durante HOLDOVER.")

        detalle_registro("Último válido ANTES", anterior)
        detalle_registro("Último HOLDOVER", durante[-1])
        detalle_registro("Primer PPS de REGRESO", regreso)
        if regreso is not None:
            if regreso.get("modo") != "PPS_REENGANCHE":
                print("  ADVERTENCIA: el primer PPS de regreso no lleva "
                      "el modo PPS_REENGANCHE; revisar el firmware/log.")
            if regreso.get("kalman_actualizado") is True:
                print("  ¡ALERTA! Se actualizó Kalman en el PPS de reenganche.")
            else:
                print("  OK: el primer PPS de regreso no actualizó Kalman.")
        detalle_registro("Primera UPDATE posterior", primera_actualizacion)
        detalle_registro("Primera comparación con GNSS", primer_comparable)

        if primer_comparable is None:
            print("No hay muestras comparables posteriores: "
                  "no se puede cuantificar el error tras recuperar PPS.")
            continue

        ms_base = primer_comparable.get("ms")
        tiempo_posterior = None
        if ms_base is not None:
            ms_validos = [r["ms"] for r in despues
                          if r.get("ms") is not None
                          and relojes_comparables(r) is not None]
            if ms_validos:
                tiempo_posterior = (max(ms_validos) - ms_base) / 1000
                print(f"  Tramo comparable posterior disponible: "
                      f"{duracion(tiempo_posterior)}")
        for minutos in (1, 5, 15, 30):
            if ms_base is None or tiempo_posterior is None:
                continue
            if tiempo_posterior < minutos * 60 - 1:
                continue
            ventana = [r for r in despues if r.get("ms") is not None
                       and 0 <= r["ms"] - ms_base <= minutos * 60000]
            comparaciones = [c for r in ventana
                             if (c := relojes_comparables(r)) is not None]
            if len(comparaciones) < 2:
                continue
            errores_k = [c[3] for c in comparaciones]
            errores_s = [c[4] for c in comparaciones]
            print(f"  Hasta {minutos:2d} min después del primer dato comparable "
                  f"({len(comparaciones)} muestras): "
                  f"Kalman MAE={fmean(abs(e) for e in errores_k):.3f} µs, "
                  f"final={errores_k[-1]:+.3f} µs; "
                  f"SQW MAE={fmean(abs(e) for e in errores_s):.3f} µs, "
                  f"final={errores_s[-1]:+.3f} µs")

        print("  IMPORTANTE: offset_ticks no se acumula durante la pérdida. "
              "El siguiente update usa el acumulador anterior más los "
              "intervalos nuevos. Esto merece revisar la continuidad de phi.")


def analizar(rutas, salida):
    por_proto = defaultdict(list)
    totales = Counter()
    modos = defaultdict(Counter)
    nulos = Counter()
    historial_proto1 = defaultdict(list)

    for ruta in rutas:
        for origen, d in registros(ruta):
            nombre = str(d.get("id") or Path(origen).stem)
            totales[nombre] += 1
            modos[nombre][str(d.get("modo", "?"))] += 1
            if "prototipo-1" in nombre:
                historial_proto1[nombre].append(d)
            try:
                gnss = instante(d["reloj_gnss_pps"])
                kalman = instante(d["reloj_kalman"])
                sqw = instante(hora_sqw(d))
                error_k = microsegundos(kalman, gnss)
                error_s = microsegundos(sqw, gnss)
                # El desfase del firmware conserva fracciones de microsegundo;
                # la hora ISO publicada tiene resolución de 1 microsegundo.
                fino = d.get("desfase_kalman_us")
                if isinstance(fino, (int, float)) and math.isfinite(fino):
                    if abs(fino - error_k) <= 1.1:
                        error_k = float(fino)
            except (KeyError, TypeError, ValueError, OverflowError):
                nulos[nombre] += 1
                continue
            por_proto[nombre].append((
                d.get("ms"), d.get("pps_seq"), gnss, kalman, sqw, error_k, error_s
            ))

    salida.mkdir(parents=True, exist_ok=True)
    for nombre in sorted(totales):
        muestras = por_proto[nombre]
        print("\n" + "=" * 65)
        print(nombre)
        print(f"Registros: {totales[nombre]}; muestras con los tres relojes: {len(muestras)}")
        print(f"Registros incompletos: {nulos[nombre]}")
        print("Modos: " + ", ".join(f"{k}={v}" for k, v in modos[nombre].items()))
        if not muestras:
            print("No hay registros con las tres horas simultáneamente.")
            continue

        primero, ultimo = muestras[0], muestras[-1]
        if primero[0] is not None and ultimo[0] is not None:
            seg = (ultimo[0] - primero[0]) / 1000
            xs = [(r[0] - primero[0]) / 3600000 for r in muestras]
        else:
            seg = (ultimo[2] - primero[2]).total_seconds()
            xs = [(r[2] - primero[2]).total_seconds() / 3600 for r in muestras]
        ek = [r[5] for r in muestras]
        es = [r[6] for r in muestras]

        print("Duración de muestras comparables:", duracion(seg))
        print("ÚLTIMAS TRES HORAS (misma muestra):")
        print("  GNSS/PPS:", ultimo[2].isoformat(sep=" ", timespec="microseconds"))
        print("  Kalman:  ", ultimo[3].isoformat(sep=" ", timespec="microseconds"))
        print("  RTC/SQW: ", ultimo[4].isoformat(sep=" ", timespec="microseconds"))
        print("  PPS secuencia:", ultimo[1])
        for etiqueta, errores in (("Kalman", ek), ("RTC/SQW", es)):
            print(f"{etiqueta}: desfase final={errores[-1]:+.3f} µs; "
                  f"MAE={fmean(abs(e) for e in errores):.3f} µs; "
                  f"P95(abs)={p95_abs(errores):.3f} µs; "
                  f"máximo(abs)={max(abs(e) for e in errores):.3f} µs")

        fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True,
                                 gridspec_kw={"height_ratios": [1.6, 1]})
        fig.suptitle(f"{nombre} — GNSS/PPS, Kalman y RTC/SQW", fontsize=16)
        for ax in axes:
            ax.axhline(0, color="#334155", ls="--", lw=1.4,
                       label="GNSS/PPS (referencia)")
            ax.plot(xs, ek, color="#7c3aed", lw=1.1, label="Kalman")
            ax.grid(alpha=0.25)
            ax.set_ylabel("Desfase respecto de GNSS (µs)")
            ax.legend(frameon=False)
        axes[0].plot(xs, es, color="#ea580c", lw=1.1, label="RTC/SQW")
        axes[0].set_title("Los tres relojes, misma escala", loc="left")
        axes[0].legend(frameon=False)
        axes[1].set_title("Ampliación de Kalman", loc="left")
        axes[1].set_xlabel("Tiempo desde la primera muestra comparable (horas)")
        fig.text(0.5, 0.01,
                 "Los desfases se calculan en cada registro; RTC/SQW no entra en Kalman.",
                 ha="center", fontsize=9)
        fig.tight_layout(rect=[0, 0.04, 1, 0.95])
        archivo = salida / ("tres_relojes_" +
                             re.sub(r"[^A-Za-z0-9_-]", "_", nombre) + ".png")
        fig.savefig(archivo, dpi=170)
        plt.close(fig)
        print("Gráfico:", archivo.resolve())

        if nombre in historial_proto1:
            informar_holdover_proto1(historial_proto1[nombre])

    # El informe de holdover también debe aparecer si no hubo ninguna
    # muestra con los tres relojes (por ejemplo, si GNSS nunca se recuperó).
    for nombre in sorted(historial_proto1):
        if nombre not in por_proto or not por_proto[nombre]:
            informar_holdover_proto1(historial_proto1[nombre])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("logs", nargs="+", type=Path, help="ZIP o archivos TXT/JSONL")
    p.add_argument("--salida", type=Path, default=Path("graficos_tres_relojes"))
    a = p.parse_args()
    for ruta in a.logs:
        if not ruta.is_file():
            p.error(f"No se encontró: {ruta}")
    analizar(a.logs, a.salida)


if __name__ == "__main__":
    main()