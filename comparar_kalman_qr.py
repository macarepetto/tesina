#!/usr/bin/env python3
"""
Comparacion Q/R del Kalman de la tesina GNSS-RTC.

Reproduce la implementacion de Kalman_Filter.cpp:
  x = [phi, f] (ticks, ticks/segundo), F=[[1,1],[0,1]], H=[1,0].
  R=r_phi, Q=diag(q_phi,q_f), P0=diag(100,1), x0=(0,0).

Lee el JSONL incluso cuando dos JSON quedaron concatenados en una linea.
No modifica el firmware ni los logs originales.

Uso:
  python3 comparar_kalman_qr.py "log_20261005_133141(3).txt"
  python3 comparar_kalman_qr.py "archivo.jsonl" --id proto1
"""

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

TICK_US = 1_000_000.0 / 32768.0
CONFIGS = {
    # Verificado para el log del 5 de octubre: reproduce kalman_phi y kalman_f
    'Original_log':       (1e-4, 1e-7, 1.0),
    # Configuracion candidata documentada despues, NO utilizada en aquel log
    'Candidata':          (1e-6, 1e-10, 1.0),
    'Candidata_R4':       (1e-6, 1e-10, 4.0),
    'Candidata_R9':       (1e-6, 1e-10, 9.0),
    'Intermedia':         (1e-5, 1e-9, 1.0),
    'Intermedia_R4':      (1e-5, 1e-9, 4.0),
    'Original_R4':        (1e-4, 1e-7, 4.0),
}

# Divisiones EXPLORATORIAS, elegidas mirando la corrida de ocho horas.
# No son cambios de regimen detectados estadisticamente.
CORTES_MIN = (100.0, 290.0)
HORIZONTES_SEG = (300, 600)  # Prediccion simulada a 5 y 10 minutos
PASO_EVAL_SEG = 300         # Evaluar desde un punto cada cinco minutos


def cargar_jsonl(path):
    decoder = json.JSONDecoder()
    registros = []
    lineas_problematicas = 0
    concatenadas = 0
    with path.open('r', encoding='utf-8-sig', errors='replace') as f:
        for linea in f:
            resto = linea.strip()
            if not resto:
                continue
            n_linea = 0
            while resto:
                try:
                    obj, fin = decoder.raw_decode(resto)
                except json.JSONDecodeError:
                    lineas_problematicas += 1
                    break
                n_linea += 1
                if isinstance(obj, dict) and obj.get('modo') in (
                    'PPS_INICIAL', 'PPS_KALMAN_Z_PHI',
                    'PPS_SIN_UPDATE_KALMAN', 'PPS_REENGANCHE', 'HOLDOVER'
                ) and isinstance(obj.get('ms'), (int, float)):
                    registros.append(obj)
                resto = resto[fin:].strip()
            if n_linea > 1:
                concatenadas += n_linea - 1
    print(f'Registros decodificados: {len(registros)}')
    print(f'JSON adicionales concatenados: {concatenadas}')
    if lineas_problematicas:
        print(f'ADVERTENCIA: fragmentos no decodificados: {lineas_problematicas}')
    return registros


def simular_kalman(rows, q_phi, q_f, r_phi):
    """Copia de la logica predict()/updatePhi() de Kalman_Filter.cpp."""
    phi, f = 0.0, 0.0
    p00, p01, p10, p11 = 100.0, 0.0, 0.0, 1.0
    phis = np.zeros(len(rows), dtype=float)
    fs = np.zeros(len(rows), dtype=float)

    for i, r in enumerate(rows):
        modo = r.get('modo', '')
        # En main_kalman: PPS normal -> predict()+updatePhi();
        # holdover -> predict() sin updatePhi(); otros -> estado inalterado.
        predecir = modo in ('PPS_KALMAN_Z_PHI', 'HOLDOVER')
        actualizar = modo == 'PPS_KALMAN_Z_PHI' and r.get('offset_ticks') is not None

        if predecir:
            phi = phi + f
            # f no cambia en la prediccion
            p00, p01, p10, p11 = (
                p00 + p01 + p10 + p11 + q_phi,
                p01 + p11,
                p10 + p11,
                p11 + q_f,
            )
        if actualizar:
            z = float(r['offset_ticks'])
            innovacion = z - phi
            s = p00 + r_phi
            k_phi = p00 / s
            k_f = p10 / s
            phi += k_phi * innovacion
            f += k_f * innovacion
            o00, o01, o10, o11 = p00, p01, p10, p11
            p00 = (1.0 - k_phi) * o00
            p01 = (1.0 - k_phi) * o01
            p10 = o10 - k_f * o00
            p11 = o11 - k_f * o01
        phis[i], fs[i] = phi, f
    return phis, fs


def rectas_por_tramos(t_min, medido_ticks, validos):
    limites = (0.0, *CORTES_MIN, float(t_min[-1]) + 0.01)
    rectas = []
    for a, b in zip(limites[:-1], limites[1:]):
        m = (t_min >= a) & (t_min < b) & validos
        if np.count_nonzero(m) < 10:
            continue
        pendiente, intercepto = np.polyfit(t_min[m], medido_ticks[m], 1)
        rectas.append((a, b, pendiente, intercepto))
    return rectas


def evaluar_prediccion(rows, phi, f, horizonte_s, cada_s=300):
    """Se finge perder el GNSS en diversos puntos; se compara la prediccion
    sin actualizaciones con el offset medido H segundos despues.
    NO es un ensayo real de corte GNSS ni una medida absoluta independiente.
    """
    valid = {int(r['pps_seq']): i for i, r in enumerate(rows)
             if r.get('modo') == 'PPS_KALMAN_Z_PHI'
             and isinstance(r.get('pps_seq'), int)
             and r.get('offset_ticks') is not None}
    if not valid:
        return np.nan, 0
    first = min(valid)
    errores = []
    for seq, i in sorted(valid.items()):
        if (seq - first) % cada_s != 0:
            continue
        j = valid.get(seq + horizonte_s)
        if j is None:
            continue
        pred_phi = phi[i] + f[i] * horizonte_s
        medido_futuro = float(rows[j]['offset_ticks'])
        errores.append(abs(pred_phi - medido_futuro) * TICK_US)
    if not errores:
        return np.nan, 0
    return float(np.mean(errores)), len(errores)


def graficar_offset(destino, t, mediciones_us, simulaciones, rectas, zoom=False):
    fig, ax = plt.subplots(figsize=(14, 6.8))
    ax.step(t, mediciones_us, where='post', label='Offset medido (cuantizado)',
            color='#737373', lw=0.9, alpha=0.55, zorder=1)
    estilos = {
        'Original_log': ('#D97706', 1.6),
        'Candidata': ('#047857', 2.2),
        'Candidata_R4': ('#2563EB', 1.8),
        'Candidata_R9': ('#9333EA', 1.5),
        'Intermedia': ('#E11D48', 1.3),
        'Intermedia_R4': ('#0891B2', 1.3),
        'Original_R4': ('#A16207', 1.3),
    }
    for nombre, (phi, f) in simulaciones.items():
        if zoom and nombre not in ('Original_log', 'Candidata', 'Candidata_R4', 'Candidata_R9'):
            continue
        color, ancho = estilos[nombre]
        ax.plot(t, phi * TICK_US, color=color, lw=ancho, alpha=0.9,
                label=nombre.replace('_', ' '), zorder=3)
    for j, (ini, fin, m, b) in enumerate(rectas):
        xs = np.array([ini, min(fin, float(t[-1]))])
        ax.plot(xs, (m*xs+b)*TICK_US, 'k--', lw=2,
                alpha=0.8, label='Tendencia por tramos (ajuste retrospectivo)' if j==0 else None)
    for cut in CORTES_MIN:
        ax.axvline(cut, color='black', linestyle=':', alpha=0.35)
    if zoom:
        ax.set_xlim(max(0, float(t[-1])-170), float(t[-1])+3)
    ax.set_title('GNSS–RTC: Kalman con distintas Q/R' + (' (detalle final)' if zoom else ' (corrida completa)'))
    ax.set_xlabel('Minutos desde el primer registro')
    ax.set_ylabel('Offset (microsegundos)')
    ax.grid(alpha=0.2)
    ax.legend(ncol=2, fontsize=9, loc='best')
    fig.tight_layout()
    fig.savefig(destino, dpi=165)
    plt.close(fig)


def graficar_deriva(destino, t, simulaciones, rectas):
    fig, ax = plt.subplots(figsize=(14, 5.5))
    colores = ['#D97706','#047857','#2563EB','#9333EA','#E11D48','#0891B2','#A16207']
    for (nombre, (_, f)), color in zip(simulaciones.items(), colores):
        ax.plot(t, f*TICK_US, lw=1.6, color=color, alpha=.85,
                label=nombre.replace('_',' '))
    for j,(ini,fin,m,b) in enumerate(rectas):
        ax.hlines(m*TICK_US/60.0, ini, min(fin,float(t[-1])),
                  linestyle='--', lw=2.5, color='black',
                  label='Pendiente por tramo (µs/s)' if j==0 else None)
    for cut in CORTES_MIN:
        ax.axvline(cut, color='black', linestyle=':', alpha=.35)
    ax.set_xlabel('Minutos desde el primer registro')
    ax.set_ylabel('Deriva f estimada (microsegundos/segundo)')
    ax.set_title('Estimacion de deriva: comparar con pendientes por tramos')
    ax.grid(alpha=.2)
    ax.legend(ncol=3, fontsize=9)
    fig.tight_layout()
    fig.savefig(destino, dpi=165)
    plt.close(fig)


def guardar_csv(path, campos, filas):
    with path.open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=campos)
        w.writeheader()
        w.writerows(filas)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('log', nargs='?', default='log_20261005_133141(3).txt',
                    help='Ruta del archivo JSONL (default: log largo del 05/10)')
    ap.add_argument('--id', dest='device_id', help='Id del dispositivo si el log mezcla dos equipos')
    ap.add_argument('--salida', default='resultados_kalman', help='Carpeta de salida')
    args = ap.parse_args()
    archivo = Path(args.log)
    if not archivo.is_file():
        raise SystemExit(f'No existe el log: {archivo.resolve()}')
    rows = cargar_jsonl(archivo)
    ids = sorted({r.get('id') for r in rows if isinstance(r.get('id'), str)})
    if args.device_id:
        rows = [r for r in rows if r.get('id') == args.device_id]
        if not rows:
            raise SystemExit(f'No hay muestras con id={args.device_id!r}; encontrados: {ids}')
    elif len(ids) > 1:
        raise SystemExit(f'Hay varios dispositivos: {ids}. Repetir con --id IDENTIFICADOR')
    rows.sort(key=lambda r: (float(r['ms']), int(r.get('pps_seq') or 0)))
    if len(rows) < 30:
        raise SystemExit('Hay muy pocas muestras de medicion.')
    t = np.array([(r['ms']-rows[0]['ms'])/60000.0 for r in rows])
    medido = np.array([float(r.get('offset_ticks') or 0) for r in rows])
    validos = np.array([r.get('modo') == 'PPS_KALMAN_Z_PHI' for r in rows])
    simulaciones = {nombre: simular_kalman(rows, *cfg) for nombre,cfg in CONFIGS.items()}
    rectas = rectas_por_tramos(t, medido, validos)
    out = Path(args.salida);out.mkdir(parents=True,exist_ok=True)

    for ini,fin,m,b in rectas:
        print(f'Recta {ini:.0f}–{fin:.1f} min: {m*60:.4f} ticks/h = {m*TICK_US*60:.3f} µs/h')

    metricas=[]
    for nombre,(qphi,qf,rphi) in CONFIGS.items():
        phi,f=simulaciones[nombre]
        e5,n5=evaluar_prediccion(rows,phi,f,300,PASO_EVAL_SEG)
        e10,n10=evaluar_prediccion(rows,phi,f,600,PASO_EVAL_SEG)
        met={
            'config':nombre, 'q_phi':qphi,'q_f':qf,'R_phi':rphi,
            'MAE_pred_5min_us':round(e5,3),'n_5min':n5,
            'MAE_pred_10min_us':round(e10,3),'n_10min':n10,
        }
        metricas.append(met)
        print(f'{nombre:18s} | 5min MAE {e5:7.3f} µs ({n5:2d} pruebas)'
              f' | 10min MAE {e10:7.3f} µs ({n10:2d})')
    guardar_csv(out/'resumen_configuraciones.csv',list(metricas[0]),metricas)

    medidas=[]
    for i,r in enumerate(rows):
        fila={'ms':r['ms'],'pps_seq':r.get('pps_seq'),'modo':r.get('modo'),
              'offset_ticks':r.get('offset_ticks'),'offset_us':r.get('offset_us')}
        for nombre,(phi,f) in simulaciones.items():
            fila[f'{nombre}_phi_us']=round(phi[i]*TICK_US,6)
            fila[f'{nombre}_f_us_s']=round(f[i]*TICK_US,9)
        medidas.append(fila)
    guardar_csv(out/'series_comparadas.csv', list(medidas[0]), medidas)
    graficar_offset(out/'01_offset_corrida_completa.png',t,medido*TICK_US,simulaciones,rectas)
    graficar_offset(out/'02_offset_detalle_final.png',t,medido*TICK_US,simulaciones,rectas,zoom=True)
    graficar_deriva(out/'03_deriva_comparada.png',t,simulaciones,rectas)

    # Comprobacion con los valores que el propio ESP32 ya guardo en el log.
    phi, f = simulaciones['Original_log']
    comp = [(i,r) for i,r in enumerate(rows)
            if r.get('modo')=='PPS_KALMAN_Z_PHI'
            and isinstance(r.get('kalman_phi'),(int,float))
            and isinstance(r.get('kalman_f'),(int,float))]
    if comp:
        max_phi = max(abs(phi[i]-r['kalman_phi']) for i,r in comp)
        max_f = max(abs(f[i]-r['kalman_f']) for i,r in comp)
        print(f'Chequeo reproduccion del firmware, diferencias maximas: '
              f'phi={max_phi:.2g} ticks, f={max_f:.2g} ticks/segundo')
        if max_phi>0.001 or max_f>0.001:
            print('AVISO: el log podria corresponder a otra configuracion, '
                  'otra version de firmware o una distinta logica de actualizacion.')
    print(f'\nArchivos guardados en: {out.resolve()}')
    print('IMPORTANTE: rectas por tramos y pseudo-holdover son analisis retrospectivos, '
          'no una verificacion de precision absoluta ni un ensayo real sin GNSS.')

if __name__=='__main__':
    main()
