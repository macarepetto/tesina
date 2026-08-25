import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

plt.style.use('seaborn-v0_8-whitegrid')

# 1. Cargar tus datos (Asegurate de usar el txt correcto)
df = pd.read_json('log_20260327_204326.txt', lines=True)

# 2. Filtrar desde el primer sync_event válido para sacar la basura inicial
indices_sync = df[df['sync_event'] == True].index
if len(indices_sync) > 0:
    df_error = df.iloc[indices_sync[0]:].dropna(subset=['error_total_us']).copy()
else:
    df_error = df.dropna(subset=['error_total_us']).copy()

# 3. Procesar las escalas de tiempo
df_error['rtc'] = pd.to_datetime(df_error['rtc'])
t_seconds = (df_error['rtc'] - df_error['rtc'].iloc[0]).dt.total_seconds().values

# Calcular tiempos en microsegundos (us)
# Tu archivo tiene el "error_total_us" que es literalmente (RTC - GPS)
error_us = df_error['error_total_us'].values

# El tiempo base del RTC en microsegundos (us)
rtc_time_us = t_seconds * 1_000_000

# El tiempo del GPS = Tiempo RTC - error
gps_time_us = rtc_time_us - error_us

# ==========================================
# GRÁFICO 1: GPS time vs RTC time
# ==========================================
fig1, ax1 = plt.subplots(figsize=(8, 5))

# Trazamos la línea de sincronización
ax1.plot(rtc_time_us, gps_time_us, color='tab:blue', linewidth=2)

ax1.set_title('GPS time vs RTC time', fontsize=14, fontweight='bold')
ax1.set_xlabel('RTC time (us)', fontweight='bold')
ax1.set_ylabel('GPS time (us)', fontweight='bold')

# Agregamos una cuadrícula suave para que se vea más técnico
ax1.grid(True, alpha=0.5)
fig1.tight_layout()
plt.savefig('gps_vs_rtc_profe.png', dpi=300)
plt.close()

# ==========================================
# GRÁFICO 2: RTC-GPS (Error a lo largo del tiempo)
# ==========================================
fig2, ax2 = plt.subplots(figsize=(8, 5))

# Acá ploteamos Tiempo(s) en el X, y el Error en el Y, tal cual pidió
ax2.plot(t_seconds, error_us, color='tab:red', linewidth=1.5)

ax2.set_title('RTC-GPS Error', fontsize=14, fontweight='bold')
ax2.set_xlabel('Time(s)', fontweight='bold')
ax2.set_ylabel('RTC-GPS (us)', fontweight='bold')

ax2.grid(True, alpha=0.5)
fig2.tight_layout()
plt.savefig('rtc_gps_error_profe.png', dpi=300)
plt.close()

print("¡Gráficos generados con los ejes exactos del profe!")