import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Configuraciones para gráficos atractivos
plt.style.use('seaborn-v0_8-whitegrid')

# 1. Cargar tus datos reales (ya como líneas JSON, usando tu nuevo txt)
df = pd.read_json('log_20260327_204326.txt', lines=True)

# 2. Procesar la columna de tiempo ("rtc")
df['rtc'] = pd.to_datetime(df['rtc'])

# Calculamos el tiempo transcurrido en horas desde el arranque
t_seconds = (df['rtc'] - df['rtc'].iloc[0]).dt.total_seconds()
df['t_hours'] = t_seconds / 3600.0

# 3. Datos para el gráfico de Satélites y PDOP (Todos los datos)
# Limpiamos solo los que no tengan info de satélites
df_sats = df.dropna(subset=['gga_sats', 'pdop'])
t_hours_sats = df_sats['t_hours'].values
sats = df_sats['gga_sats'].values
pdop = df_sats['pdop'].values

# ==========================================
# GRÁFICO 1: Satélites vs PDOP (Todos los datos)
# ==========================================
fig1, ax1 = plt.subplots(figsize=(8, 4.5))

color1 = 'tab:blue'
ax1.set_xlabel('Tiempo de operación (Horas)', fontweight='bold')
ax1.set_ylabel('Satélites Visibles (GGA)', color=color1, fontweight='bold')
ax1.plot(t_hours_sats, sats, color=color1, linewidth=2, label='Satélites Visibles')
ax1.tick_params(axis='y', labelcolor=color1)
# Margen dinámico superior
ax1.set_ylim(0, max(sats) + 2 if len(sats) > 0 else 14) 

# Segundo eje Y para el PDOP
ax2 = ax1.twinx()  
color2 = 'tab:red'
ax2.set_ylabel('PDOP (Dilución de Precisión)', color=color2, fontweight='bold')
ax2.plot(t_hours_sats, pdop, color=color2, alpha=0.7, linewidth=1.5, label='PDOP')
ax2.tick_params(axis='y', labelcolor=color2)
ax2.set_ylim(0, max(pdop) + 2 if len(pdop) > 0 else 6)

plt.title('Calidad de Señal GNSS Real: Satélites y PDOP', fontsize=14, fontweight='bold')
fig1.tight_layout()
plt.savefig('satelites_pdop_real.png', dpi=300)
plt.close()

# 4. Datos para el gráfico de Error (A partir del primer sync_event)
# Buscamos en qué índice (fila) ocurre el primer sync_event == True
indices_sync = df[df['sync_event'] == True].index

if len(indices_sync) > 0:
    first_sync_idx = indices_sync[0]
    # Recortamos el dataframe desde ese punto en adelante
    df_error = df.iloc[first_sync_idx:].copy()
else:
    # Si por algún motivo no hay sync_event en este log, usamos todo
    df_error = df.copy()

# Filtramos usando TU nombre de variable: error_total_us
df_error = df_error.dropna(subset=['error_total_us'])

t_hours_error = df_error['t_hours'].values
error_real_us = df_error['error_total_us'].values

# ==========================================
# GRÁFICO 2: Offset RTC Libre vs Tu Sistema (Post-Sync)
# ==========================================
# Deriva libre teórica (2 ppm) para contrastar
drift_rate = 2.0 
offset_libre_us = drift_rate * (t_hours_error * 3600) 

fig2, ax = plt.subplots(figsize=(8, 4.5))

ax.plot(t_hours_error, offset_libre_us / 1000, color='tab:red',linewidth=2)
#ax.plot(t_hours_error, error_real_us / 1000, color='tab:green', linewidth=2, label='RTC Disciplinado')

ax.set_xlabel('Tiempo de operación (Horas)', fontweight='bold')
ax.set_ylabel('Error de Sincronización (Milisegundos)', fontweight='bold')
ax.set_title('Error de Sincronización RTC y GPS', fontsize=14, fontweight='bold')
#ax.legend(loc='upper right', fontsize=11)

# Inset plot (Zoom) para mostrar tus microsegundos reales
# from mpl_toolkits.axes_grid1.inset_locator import inset_axes
# axins = inset_axes(ax, width="40%", height="40%", loc=5, borderpad=3)
# axins.plot(t_hours_error, error_real_us, color='tab:green', linewidth=1, alpha=0.8)
# axins.set_title('Zoom RTC Disciplinado (μs)', fontsize=9)
# axins.set_ylabel('Microsegundos', fontsize=8)
# axins.tick_params(axis='both', which='major', labelsize=8)
# axins.grid(True, alpha=0.5)

fig2.tight_layout()
plt.savefig('error_rtc_real.png', dpi=300)
plt.close()

# ==========================================
# GRÁFICO 3: Histograma de Jitter de tus mediciones
# ==========================================
fig3, ax3 = plt.subplots(figsize=(6, 4.5))
ax3.hist(error_real_us, bins=40, color='tab:green', alpha=0.7, edgecolor='black')
ax3.set_xlabel('Error de sincronización (Microsegundos)', fontweight='bold')
ax3.set_ylabel('Frecuencia (Cantidad de Muestras)', fontweight='bold')
ax3.set_title('Distribución del Error Empírico', fontsize=14, fontweight='bold')
fig3.tight_layout()
plt.savefig('jitter_histogram_real.png', dpi=300)
plt.close()

print("¡Gráficos generados con éxito aplicando el filtro de sync_event!")