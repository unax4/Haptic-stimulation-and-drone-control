import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# --- Configuración ---
FOV_DEG = 63          # Campo de visión en grados
FOV_RAD = np.radians(FOV_DEG / 2)
N_CIRCLE_PTS = 500      

cube_faces = {
    "Norte":  (  0.0,  90.0),
    "Sur":    (  0.0, -90.0),
    "Este":   ( 90.0,   0.0),
    "Oeste":  (-90.0,   0.0),
    "Frente": (  0.0,   0.0),
    "Atrás":  (180.0,   0.0),
}

COLORS = ['#E63946', '#457B9D', '#2A9D8F', '#E9C46A', '#F4A261', '#8338EC']

def great_circle_fov(center_lon_deg, center_lat_deg, fov_radius_rad, n=N_CIRCLE_PTS):
    lon0, lat0 = np.radians(center_lon_deg), np.radians(center_lat_deg)
    az = np.linspace(0, 2 * np.pi, n)

    # Coordenadas locales respecto al polo
    x_l = np.sin(fov_radius_rad) * np.cos(az)
    y_l = np.sin(fov_radius_rad) * np.sin(az)
    z_l = np.cos(fov_radius_rad) * np.ones_like(az)

    # Rotación en Y (latitud)
    angle_tilt = np.pi / 2 - lat0
    cos_t, sin_t = np.cos(angle_tilt), np.sin(angle_tilt)
    x_t, y_t, z_t = cos_t * x_l + sin_t * z_l, y_l, -sin_t * x_l + cos_t * z_l

    # Rotación en Z (longitud)
    cos_l, sin_l = np.cos(lon0), np.sin(lon0)
    x_f, y_f, z_f = cos_l * x_t - sin_l * y_t, sin_l * x_t + cos_l * y_t, z_t

    return np.arctan2(y_f, x_f), np.arcsin(np.clip(z_f, -1, 1))

def split_at_boundary(lon_pts, lat_pts, threshold=np.pi):
    segments = []
    curr_lon, curr_lat = [lon_pts[0]], [lat_pts[0]]
    for i in range(1, len(lon_pts)):
        if abs(lon_pts[i] - lon_pts[i-1]) > threshold:
            segments.append((np.array(curr_lon), np.array(curr_lat)))
            curr_lon, curr_lat = [], []
        curr_lon.append(lon_pts[i])
        curr_lat.append(lat_pts[i])
    if curr_lon: segments.append((np.array(curr_lon), np.array(curr_lat)))
    return segments

# --- Figura ---
fig = plt.figure(figsize=(12, 6), facecolor='#0d1117')
ax = fig.add_subplot(111, projection='mollweide')
ax.set_facecolor('#0d1117')
ax.grid(True, color='#30363d', linewidth=0.5, linestyle='--', alpha=0.5)

for (name, (lon_deg, lat_deg)), color in zip(cube_faces.items(), COLORS):
    lon_pts, lat_pts = great_circle_fov(lon_deg, lat_deg, FOV_RAD)
    
    # Manejo especial para Polos (Norte/Sur) para asegurar el relleno completo
    if name in ["Norte", "Sur"]:
        # Ordenamos por longitud para que el fill no haga formas extrañas
        sort_idx = np.argsort(lon_pts)
        lons, lats = lon_pts[sort_idx], lat_pts[sort_idx]
        
        # Para el Norte, el relleno debe incluir el límite superior (pi/2)
        # Para el Sur, el límite inferior (-pi/2)
        pole_lat = np.pi/2 if name == "Norte" else -np.pi/2
        
        # Creamos un polígono que recorre el arco y cierra en los bordes del mapa
        fill_lon = np.concatenate([[-np.pi], lons, [np.pi]])
        fill_lat = np.concatenate([[pole_lat], lats, [pole_lat]])
        
        ax.fill(fill_lon, fill_lat, color=color, alpha=0.2)
        ax.plot(lons, lats, color=color, linewidth=2, alpha=0.8)
    else:
        # Lógica normal para caras ecuatoriales
        segments = split_at_boundary(lon_pts, lat_pts)
        for s_lon, s_lat in segments:
            ax.plot(s_lon, s_lat, color=color, linewidth=2, alpha=0.8)
            ax.fill(s_lon, s_lat, color=color, alpha=0.2)

    # Punto central y etiqueta
    lon_c, lat_c = np.radians(lon_deg), np.radians(lat_deg)
    # Ajuste para el punto de la cara "Atrás" que está en el borde
    if name == "Atrás": lon_c = np.pi 

    ax.plot(lon_c, lat_c, 'o', color=color, markersize=7, markeredgecolor='white', zorder=5)
    ax.annotate(name, xy=(lon_c, lat_c), xytext=(0, 10), textcoords='offset points',
                color=color, fontsize=10, fontweight='bold', ha='center',
                bbox=dict(boxstyle='round,pad=0.1', fc='#0d1117', ec=color, alpha=0.6))

# Estética final
ax.tick_params(colors='#8b949e', labelsize=8)
ax.set_title(f"Proyección Mollweide - FOV {FOV_DEG}°", color='white', pad=20)

plt.tight_layout()
plt.show()