#!/usr/bin/env python3
"""Compara variantes Q/R del Kalman GNSS–RTC sobre un JSONL real.

Necesita tener comparar_kalman_qr.py en la misma carpeta.
Uso: python3 comparar_kalman_qr_claro.py 'log_20261005_133141(3).txt'
Dependencias: numpy, matplotlib

Las predicciones a 5/10 minutos son retrospectivas, NO prueban precision absoluta.
"""
import argparse
import csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from comparar_kalman_qr import cargar_jsonl, simular_kalman, evaluar_prediccion, TICK_US

# Variantes para EXPLORAR. No son automaticamente mejores.
CONFIGS = {
    'Original': (1e-4, 1e-7, 1),
    'Candidata': (1e-6, 1e-10, 1),
    'Candidata_R4': (1e-6, 1e-10, 4),
    'Candidata_Qphi_mas_alto': (1e-5, 1e-10, 1),
    'Candidata_Qphi_intermedio': (3e-6, 1e-10, 1),
    'Candidata_Qf_mas_lento': (1e-6, 3e-11, 1),
    'Candidata_Qf_mas_rapido': (1e-6, 3e-10, 1),
}
# Para evitar graficos cargados, solo se grafican estas tres de forma predeterminada.
MOSTRAR = ('Original', 'Candidata', 'Candidata_R4')
COLORES = {'Original': '#B45309', 'Candidata': '#2563EB', 'Candidata_R4': '#047857'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log',type=Path)
    parser.add_argument('--salida',type=Path,default=Path('comparacion_clara'))
    parser.add_argument('--minuto-inicial',type=float,default=290,
                        help='Zoom: min desde el primer registro. Usar 0 para corrida corta')
    args=parser.parse_args()
    if not args.log.is_file():parser.error('No se encuentra el archivo de log')
    registros=cargar_jsonl(args.log)
    registros=[r for r in registros if r.get('modo') in ('PPS_INICIAL','PPS_KALMAN_Z_PHI','PPS_SIN_UPDATE_KALMAN','PPS_REENGANCHE','HOLDOVER')]
    ids={r.get('id') for r in registros}
    if len(ids)>1:parser.error('Log con mas de un dispositivo; separar por id antes de analizar')
    registros.sort(key=lambda r: (float(r['ms']),int(r.get('pps_seq')or 0)))
    if len(registros)<30:parser.error('Muy pocos registros validos')
    t=np.asarray([(r['ms']-registros[0]['ms'])/60000 for r in registros])
    offset=np.asarray([float(r.get('offset_ticks') or 0)*TICK_US for r in registros])
    t0=max(0,float(args.minuto_inicial));mask=t>=t0
    if np.count_nonzero(mask)<20:parser.error('Minuto inicial demasiado alto para este log')
    simulaciones={name:simular_kalman(registros,*config) for name,config in CONFIGS.items()}
    args.salida.mkdir(parents=True,exist_ok=True)

    # Guardar tabla numerica: todas las variantes, evaluadas con las mismas muestras.
    resultados=[]
    for name, (qphi,qf,R) in CONFIGS.items():
        phi,f=simulaciones[name]
        mae5,n5=evaluar_prediccion(registros,phi,f,300,300)
        mae10,n10=evaluar_prediccion(registros,phi,f,600,300)
        resultados.append({'config':name,'q_phi':qphi,'q_f':qf,'R_phi':R,
                           'MAE_pred_5min_us':round(mae5,4),'n_5min':n5,
                           'MAE_pred_10min_us':round(mae10,4),'n_10min':n10})
    with (args.salida/'metricas.csv').open('w',newline='',encoding='utf8') as file:
        writer=csv.DictWriter(file,list(resultados[0]));writer.writeheader();writer.writerows(resultados)
    for row in resultados:
        print(f"{row['config']:29s} 5 min: {row['MAE_pred_5min_us']:.2f} us; 10 min: {row['MAE_pred_10min_us']:.2f} us")

    fig,ax=plt.subplots(figsize=(13,5.4))
    ax.step(t[mask],offset[mask],where='post',lw=.85,alpha=.48,color='#6B7280',label='Offset medido (cuantizado)')
    for name in MOSTRAR:
        phi,_=simulaciones[name]
        ax.plot(t[mask],phi[mask]*TICK_US,lw=1.6,color=COLORES[name],label=name)
    ax.set(xlabel='Tiempo desde inicio (min)',ylabel='Offset (µs)',title='Mismas mediciones: original, candidata y R4')
    ax.legend();ax.grid(alpha=.18);fig.tight_layout()
    fig.savefig(args.salida/'01_offset_tres_curvas.png',dpi=165);plt.close(fig)

    fig,ax=plt.subplots(figsize=(13,5.4))
    for name in MOSTRAR:
        _,f=simulaciones[name]
        ax.plot(t[mask],f[mask]*TICK_US,lw=1.6,color=COLORES[name],label=name)
    ax.axhline(0,linestyle='--',lw=.9)
    ax.set(xlabel='Tiempo desde inicio (min)',ylabel='Deriva estimada f (µs/s)',title='Comparacion de la deriva estimada')
    ax.legend();ax.grid(alpha=.18);fig.tight_layout()
    fig.savefig(args.salida/'02_deriva_tres_curvas.png',dpi=165);plt.close(fig)

    fig,ax=plt.subplots(figsize=(13,5.4))
    for name in MOSTRAR:
        phi,_=simulaciones[name]
        ax.plot(t[mask],(offset-phi*TICK_US)[mask],lw=1.2,color=COLORES[name],label=name)
    ax.axhline(0,linestyle='--',lw=.9)
    ax.set(xlabel='Tiempo desde inicio (min)',ylabel='Offset medido - estimado (µs)',title='Residual: mas suave NO equivale a menor residual')
    ax.legend();ax.grid(alpha=.18);fig.tight_layout()
    fig.savefig(args.salida/'03_residual_tres_curvas.png',dpi=165);plt.close(fig)
    print('\nArchivos guardados en',args.salida.resolve())

if __name__=='__main__':main()
