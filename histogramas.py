import pandas as pd
import matplotlib.pyplot as plt

# Configuraciones de diseño
plt.style.use('seaborn-v0_8-whitegrid')

# 1. Cargar tus datos (Asegurate de usar el txt correcto)
df = pd.read_json('log_20260327_204326.txt', lines=True)

# 2. Filtrar los datos válidos de satélites y PDOP
df_sats = df.dropna(subset=['gga_sats', 'pdop'])

sats = df_sats['gga_sats']
pdop = df_sats['pdop']

# ==========================================
# HISTOGRAMA 1: Satélites Visibles
# ==========================================
fig1, ax1 = plt.subplots(figsize=(6, 4.5))

# Como los satélites son enteros, armamos cajas exactas
bins_sats = range(int(sats.min()), int(sats.max()) + 2)

ax1.hist(sats, bins=bins_sats, color='tab:blue', alpha=0.7, edgecolor='black', align='left')

ax1.set_xlabel('Cantidad de Satélites Visibles', fontweight='bold')
ax1.set_ylabel('Frecuencia (Cantidad de Muestras)', fontweight='bold')
ax1.set_title('Distribución de Satélites Visibles', fontsize=14, fontweight='bold')

# Forzamos a que el eje X muestre solo números enteros
ax1.set_xticks(range(int(sats.min()), int(sats.max()) + 1))

# Un truquito de diseño: la grilla la ponemos solo horizontal
ax1.grid(axis='x', visible=False)

fig1.tight_layout()
plt.savefig('histograma_satelites.png', dpi=300)
plt.close()

# ==========================================
# HISTOGRAMA 2: PDOP
# ==========================================
fig2, ax2 = plt.subplots(figsize=(6, 4.5))

# El PDOP es un valor continuo (con decimales), usamos 20 divisiones
ax2.hist(pdop, bins=20, color='tab:red', alpha=0.7, edgecolor='black')

ax2.set_xlabel('Valor de PDOP (Dilución de Precisión)', fontweight='bold')
ax2.set_ylabel('Frecuencia (Cantidad de Muestras)', fontweight='bold')
ax2.set_title('Distribución del PDOP', fontsize=14, fontweight='bold')

ax2.grid(axis='x', visible=False)

fig2.tight_layout()
plt.savefig('histograma_pdop.png', dpi=300)
plt.close()

print("¡Histogramas generados con éxito!")