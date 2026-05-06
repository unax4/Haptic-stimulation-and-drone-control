import threading
import numpy as np
import pybullet as p
import pybullet_data
import time
import serial 
from collections import deque
import tkinter as tk
from queue import Queue

# --- CONFIGURACIÓN SERIAL ---
SERIAL_PORT = "COM3"
BAUD_RATE = 115200

# --- CONSTANTES ---
MIN_FLEXION_RAD = 0.0
MAX_FLEXION_RAD = 2 
TRANSLATION_SPEED = 0.01 

# --- CONFIGURACIÓN DE CUBO FIJO ---
CUBE_SIZE = 0.3  
CUBE_POSITION = [0.3, 0.0, CUBE_SIZE/2]  # Fijo en el suelo

# Colas
data_queue = deque(maxlen=1)
command_queue = deque()
gui_queue = Queue()
running = True

def gui_print(msg):
    gui_queue.put(("output", msg))

class HapticConfig:
    """Configuración de feedback háptico para cada zona de colisión"""
    def __init__(self):
        # Mapeo: zona -> list of {'M': int, 'stim_type': str, 'stim_params': dict}
        # Tipos: 'S' = single, 'T' = train, 'B' = burst
        self.zone_haptic_map = {
            # Mano de 5 dedos (con proximal y distal separados, y múltiples M por segmento)
            'PALM': [
                {'M': 19, 'stim_type': 'B', 'stim_params': {}},
                #{'M': 20, 'stim_type': 'B', 'stim_params': {}}
            ],
            'THUMB': [
                {'M': 1, 'stim_type': 'T', 'stim_params': {}},
                {'M': 2, 'stim_type': 'B', 'stim_params': {}}
            ],  
            'INDEX_BOTTOM': [
                #{'M': 11, 'stim_type': 'T', 'stim_params': {}},
                {'M': 12, 'stim_type': 'T', 'stim_params': {}}
            ],
            'INDEX_TOP': [
                {'M': 3, 'stim_type': 'T', 'stim_params': {}},
                #{'M': 4, 'stim_type': 'T', 'stim_params': {}}
            ],
            'MIDDLE_BOTTOM': [
                #{'M': 13, 'stim_type': 'T', 'stim_params': {}},
                {'M': 14, 'stim_type': 'T', 'stim_params': {}}
            ],
            'MIDDLE_TOP': [
                {'M': 5, 'stim_type': 'T', 'stim_params': {}},
                #{'M': 6, 'stim_type': 'T', 'stim_params': {}}
            ],
            'RING_BOTTOM': [
                #{'M': 15, 'stim_type': 'T', 'stim_params': {}},
                {'M': 16, 'stim_type': 'T', 'stim_params': {}}
            ],
            'RING_TOP': [
                {'M': 7, 'stim_type': 'T', 'stim_params': {}},
                #{'M': 8, 'stim_type': 'T', 'stim_params': {}}
            ],
            'PINKY_BOTTOM': [
                #{'M': 17, 'stim_type': 'T', 'stim_params': {}},
                {'M': 18, 'stim_type': 'T', 'stim_params': {}}
            ],
            'PINKY_TOP': [
                {'M': 9, 'stim_type': 'T', 'stim_params': {}},
                #{'M': 10, 'stim_type': 'T', 'stim_params': {}}
            ],
            
        }
    
    def get_haptic_commands(self, zone):
        """Retorna los comandos para activar el feedback háptico de una zona"""
        if zone not in self.zone_haptic_map:
            return []
        
        configs = self.zone_haptic_map[zone]
        commands = []
        
        for config in configs:
            # Cambiar a la posición M correspondiente
            commands.append(f"M{config['M']}")
            
            # Enviar el estímulo correspondiente
            stim_type = config['stim_type']
            if stim_type == 'S':
                commands.append('S')  # Single pulse
            elif stim_type == 'T':
                commands.append('T')  # Train
            elif stim_type == 'B':
                commands.append('B')  # Burst
        
        return commands
    
    def get_stop_commands(self, zone):
        """Retorna los comandos para detener el feedback háptico"""
        return ['STOP']
    
    def configure_zone(self, zone, m_pos, stim_type='T', **stim_params):
        """Permite reconfigurar dinámicamente una zona (aplica al primer config si múltiple)"""
        if zone in self.zone_haptic_map:
            self.zone_haptic_map[zone][0]['M'] = m_pos
            self.zone_haptic_map[zone][0]['stim_type'] = stim_type
            self.zone_haptic_map[zone][0]['stim_params'] = stim_params
            gui_print(f"[CONFIG] Zona {zone} → M{m_pos}, Estímulo: {stim_type} (primer config)")

# ==============================================================================
# CLASE IMU: FUSIÓN DE SENSORES CON RESET RELATIVO 
# ==============================================================================
class IMUSensorFusion:
    """Clase para la fusión de sensores IMU usando filtro Mahony con calibración y offset para reset de orientación."""
    def __init__(self):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # Cuaternión inicial
        self.Kp = 5.0   # Ganancia proporcional
        self.Ki = 0.02  # Ganancia integral
        self.eInt = np.array([0.0, 0.0, 0.0])  # Error integral
        self.gyro_bias = np.array([0.0, 0.0, 0.0])  # Bias del giroscopio
        self.calibrated = False  # Estado de calibración
        self.bias_samples = []  # Muestras para calibrar bias
        
        # --- Offset para resetear orientación relativa ---
        self.q_offset = np.array([1.0, 0.0, 0.0, 0.0])

    def calibrate(self, gx, gy, gz, max_samples=100):
        """Calibra el bias del giroscopio recolectando muestras.
        gx, gy, gz: Valores del giroscopio en rad/s (post-mapeo).
        """
        self.bias_samples.append([gx, gy, gz])
        if len(self.bias_samples) >= max_samples:
            self.gyro_bias = np.mean(self.bias_samples, axis=0)
            self.calibrated = True
            self.reset_orientation()  # Reset inicial al terminar calibración
            gui_print(f"[IMU] Calibrado. Bias detectado: {self.gyro_bias}")
            return True
        return False

    def reset_orientation(self):
        """Captura la rotación actual como offset para orientación relativa."""
        # Invertimos el cuaternión actual para usarlo como offset [w, -x, -y, -z]
        self.q_offset = np.array([self.q[0], -self.q[1], -self.q[2], -self.q[3]])
        gui_print(">>> ORIENTACIÓN RECENTRADA (Tecla O) <<<")

    def update(self, ax, ay, az, gx, gy, gz, dt):
        """Actualiza el cuaternión usando datos del acelerómetro y giroscopio
        ax, ay, az: Aceleración normalizada.
        gx, gy, gz: Velocidad angular en rad/s (post-bias).
        dt: Delta tiempo.       
        """

        #añadir offsett aqui?


        if self.calibrated:
            gx -= self.gyro_bias[0]
            gy -= self.gyro_bias[1]
            gz -= self.gyro_bias[2]

        q = self.q
        norm_a = np.sqrt(ax*ax + ay*ay + az*az)
        if norm_a == 0.0: return 
        ax /= norm_a; ay /= norm_a; az /= norm_a

        # Cálculo de vectores de referencia y error
        vx = 2.0 * (q[1]*q[3] - q[0]*q[2])
        vy = 2.0 * (q[0]*q[1] + q[2]*q[3])
        vz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]

        ex = (ay * vz - az * vy); ey = (az * vx - ax * vz); ez = (ax * vy - ay * vx)

        self.eInt[0] += ex * self.Ki * dt
        self.eInt[1] += ey * self.Ki * dt
        self.eInt[2] += ez * self.Ki * dt

        gx += self.Kp * ex + self.eInt[0]
        gy += self.Kp * ey + self.eInt[1]
        gz += self.Kp * ez + self.eInt[2]

        # Integración del cuaternión
        pa, pb, pc = q[0], q[1], q[2]
        q[0] += (-q[1] * gx - q[2] * gy - q[3] * gz) * (0.5 * dt)
        q[1] += (pa * gx + q[2] * gz - q[3] * gy) * (0.5 * dt)
        q[2] += (pa * gy - pb * gz + q[3] * gx) * (0.5 * dt)
        q[3] += (pa * gz + pb * gy - pc * gx) * (0.5 * dt)

        # Normalizar cuaternión
        self.q = q / np.linalg.norm(q)
        return self.q

    def get_pybullet_quaternion(self):
        """Calcula y retorna el cuaternión relativo al offset en formato PyBullet [x, y, z, w]."""
        qo = self.q_offset
        qa = self.q
        # Multiplicación de cuaterniones (qo * qa) para obtener rotación relativa
        res = np.array([
            qo[0]*qa[0] - qo[1]*qa[1] - qo[2]*qa[2] - qo[3]*qa[3],
            qo[0]*qa[1] + qo[1]*qa[0] + qo[2]*qa[3] - qo[3]*qa[2],
            qo[0]*qa[2] - qo[1]*qa[3] + qo[2]*qa[0] + qo[3]*qa[1],
            qo[0]*qa[3] + qo[1]*qa[2] - qo[2]*qa[1] + qo[3]*qa[0]
        ])
        # Formato PyBullet: [x, y, z, w]
        return [res[1], res[2], res[3], res[0]]

class HandSimulator:
    def __init__(self, urdf_filename="schunk_svh_hand_right.urdf"):
        self.urdf_filename = urdf_filename
        self.hand_id = -1
        self.cube_id = -1
        self.num_fingers_model = 0
        self.joint_indices = {}
        self.link_indices = {}
        self.finger_mapping = {}
        self.collision_zones = {}
        self.last_collision_state = {}
        self.active_haptic_zones = set()  # Zonas con feedback activo
        
        # Configuración de haptics
        self.haptic_config = HapticConfig()
        
        # --- IMU: Usamos la clase de fusión de sensores de pruebas.py ---
        self.imu = IMUSensorFusion()
        self.calibration_steps = 150  # Número de muestras para calibrar el IMU
        
        # --- VARIABLES PARA POSICIÓN Y SUAVIZADO ---
        self.base_target_pos = np.array([0.0, 0.0, 0.3]) 
        self.translation_vector = [0.0, 0.0, 0.0]
        self.smoothing_factor = 0.3 
        self.last_time = time.time()

        # --- VARIABLES PARA FLEXión ---
        self.flex_min = np.full(self.num_fingers_model, np.inf)
        self.flex_max = np.full(self.num_fingers_model, -np.inf)

        # Teclas para mover la base de la mano
        self.key_map = {
            ord('1'): (1, 0, 0),    # +X
            ord('2'): (-1, 0, 0),   # -X
            ord('3'): (0, 1, 0),    # +Y
            ord('4'): (0, -1, 0),   # -Y
            ord('5'): (0, 0, 1),    # +Z
            ord('6'): (0, 0, -1),   # -Z
        }

        # Definición de Joints
        self.five_finger_joints = {
            'PINKY': ['right_hand_Pinky', 'right_hand_j13', 'right_hand_j17'],
            'THUMB_INDEX': ['right_hand_Thumb_Opposition', 'right_hand_Thumb_Flexion', 'right_hand_j3', 'right_hand_j4',
                            'right_hand_Index_Finger_Proximal', 'right_hand_Index_Finger_Distal', 'right_hand_j14'],
            'MIDDLE': ['right_hand_Middle_Finger_Proximal', 'right_hand_Middle_Finger_Distal', 'right_hand_j15'],
            'RING': ['right_hand_Ring_Finger', 'right_hand_j12', 'right_hand_j16'],
        }

        self.five_collision_zones = {
            'PALM': ['right_hand_base_link', 'right_hand_e1', 'right_hand_e2'],
            'THUMB': ['right_hand_z', 'right_hand_a', 'right_hand_b','right_hand_c'], 
            'INDEX_BOTTOM': ['right_hand_virtual_l', 'right_hand_l', 'right_hand_p'], 
            'INDEX_TOP': ['right_hand_t'], 
            'MIDDLE_BOTTOM': ['right_hand_virtual_k', 'right_hand_k', 'right_hand_o'], 
            'MIDDLE_TOP': ['right_hand_s'], 
            'RING_BOTTOM': ['right_hand_virtual_j', 'right_hand_j', 'right_hand_n'], 
            'RING_TOP': ['right_hand_r'], 
            'PINKY_BOTTOM': ['right_hand_virtual_i', 'right_hand_i', 'right_hand_m'], 
            'PINKY_TOP': ['right_hand_q']
        }

    def setup_pybullet(self):
        """Configura el entorno de PyBullet: conecta, carga modelos y configura visualización."""
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, 0)  # Desactivar gravedad para que no afecte a la mano
        p.setTimeStep(1. / 240.)
        
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        p.resetDebugVisualizerCamera(cameraDistance=0.8, cameraYaw=180, cameraPitch=-30, 
                                      cameraTargetPosition=[0.3, 0, 0.2])

        # Cargar plano
        p.loadURDF("plane.urdf")
        
        # CUBO FIJO Y MÁS GRANDE
        cube_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[CUBE_SIZE/2]*3)
        cube_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[CUBE_SIZE/2]*3, 
                                          rgbaColor=[0.2, 0.6, 0.8, 1])
        
        self.cube_id = p.createMultiBody(
            baseMass=0,  # Masa 0 = objeto estático (fijo)
            baseCollisionShapeIndex=cube_collision,
            baseVisualShapeIndex=cube_visual,
            basePosition=CUBE_POSITION
        )
        
        # Asegurar que el cubo esté completamente fijo
        p.changeDynamics(self.cube_id, -1, 
                        mass=0,
                        lateralFriction=1.0,
                        spinningFriction=0.1,
                        rollingFriction=0.1)

        # Cargar mano
        try:
            self.hand_id = p.loadURDF(self.urdf_filename, self.base_target_pos, 
                                     p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=False)
        except:
            gui_print("Error cargando URDF.")
            return -1

        # Indexar joints y links
        num_joints = p.getNumJoints(self.hand_id)
        for i in range(num_joints):
            info = p.getJointInfo(self.hand_id, i)
            self.joint_indices[info[1].decode('utf-8')] = i
            self.link_indices[info[12].decode('utf-8')] = i


        self.finger_mapping = self.five_finger_joints
        self.num_fingers_model = 5
        self.collision_zones = {k: [self.link_indices.get(l, -1) for l in v] 
                            for k, v in self.five_collision_zones.items()}

        self.last_collision_state = {k: False for k in self.collision_zones}
        self.reset_calibration()

        # Control de motores
        joint_uids = list(self.joint_indices.values())
        p.setJointMotorControlArray(self.hand_id, joint_uids, p.POSITION_CONTROL, 
                                    forces=[500]*len(joint_uids))
        
        # Inicializar potenciómetro al máximo
        command_queue.append("P255")
        
        gui_print(f"\n[SETUP] Mano cargada: {self.num_fingers_model} dedos")
        gui_print(f"[SETUP] Zonas de colisión: {list(self.collision_zones.keys())}")
        gui_print(f"[SETUP] Cubo fijo en {CUBE_POSITION}, tamaño: {CUBE_SIZE}m\n")
        
        return self.hand_id

    def reset_calibration(self):
        """Resetea los rangos mínimos y máximos de los sensores de flexión."""
        gui_print("[INFO] Recalibrando sensores flex...")
        self.flex_min = np.full(self.num_fingers_model, np.inf)
        self.flex_max = np.full(self.num_fingers_model, -np.inf)

    def reset_orientation(self):
        """Llama al reset de orientación del IMU para recentrar la mano."""
        self.imu.reset_orientation()

    def handle_keyboard_input(self):
        """Maneja entradas de teclado para traducción y resets."""
        events = p.getKeyboardEvents()
        self.translation_vector = [0.0, 0.0, 0.0]

        for key, value in self.key_map.items():
            if key in events and events[key] & p.KEY_IS_DOWN:
                self.translation_vector[0] += value[0] * TRANSLATION_SPEED
                self.translation_vector[1] += value[1] * TRANSLATION_SPEED
                self.translation_vector[2] += value[2] * TRANSLATION_SPEED
        
        if ord('r') in events and events[ord('r')] & p.KEY_WAS_TRIGGERED:
            self.reset_calibration()
        if ord('o') in events and events[ord('o')] & p.KEY_WAS_TRIGGERED:
            self.reset_orientation()

    def check_collisions(self):
        """Detecta colisiones con el cubo y gestiona feedback háptico."""
        contact_points = p.getContactPoints(self.hand_id, self.cube_id)
        
        # Cambiar color del cubo según si hay contacto
        if contact_points:
            p.changeVisualShape(self.cube_id, -1, rgbaColor=[1.0, 0.3, 0.0, 1.0])
        else:
            p.changeVisualShape(self.cube_id, -1, rgbaColor=[0.2, 0.6, 0.8, 1])
        
        # Detectar qué zonas están tocando
        touched_zones = {k: False for k in self.collision_zones}
        any_stop = False
        for cp in contact_points:
            link_index = cp[3]  # Link de la mano que colisiona
            for zone, links in self.collision_zones.items():
                if link_index in links:
                    touched_zones[zone] = True
                    # No break, permitir múltiples zonas por link
        
        # Procesar cambios de estado de colisión
        for zone in self.collision_zones:
            # INICIO DE COLISIÓN
            if touched_zones[zone] and not self.last_collision_state[zone]:
                gui_print(f"\n[COLLISION] ✓ Zona {zone} tocando cubo")
                
                # Enviar comandos de activación
                haptic_commands = self.haptic_config.get_haptic_commands(zone)
                for cmd in haptic_commands:
                    command_queue.append(cmd)
                    gui_print(f"[HAPTIC] → {cmd}")
                
                self.active_haptic_zones.add(zone)
            
            # FIN DE COLISIÓN
            elif not touched_zones[zone] and self.last_collision_state[zone]:
                gui_print(f"\n[COLLISION] ✗ Zona {zone} dejó de tocar")
                self.active_haptic_zones.discard(zone)
                any_stop = True
        
        # Si hubo algún stop, detener todo y reiniciar los activos restantes
        if any_stop:
            command_queue.append('STOP')
            gui_print(f"[HAPTIC] → STOP")
            for zone in self.active_haptic_zones:
                haptic_commands = self.haptic_config.get_haptic_commands(zone)
                for cmd in haptic_commands:
                    command_queue.append(cmd)
                    gui_print(f"[HAPTIC] → {cmd} (reinicio)")
        
        self.last_collision_state = touched_zones

    def process_sensor_data(self, line):
        """Procesa datos del Arduino: time, A3, A2, A1, A0, ax, ay, az, gx, gy, gz.
        Integra manejo de IMU y orientación de pruebas.py.
        """
        parts = line.split(",")
        if len(parts) < 11: 
            return False
        
        try:
            _, *vals = [float(v) for v in parts[:11]]
            
            # Mapeo de analógicos: [A0, A3, A2, A1] (mantener orden de main_prog_v1.py)
            analogs_raw = [vals[3], vals[0], vals[1], vals[2]]
            
            # IMU raw values
            ax, ay, az, gx, gy, gz = vals[4:10]
        except ValueError: 
            return False

        dt = time.time() - self.last_time
        self.last_time = time.time()

        # --- Mapeo de ejes IMU (igual que en main y pruebas) ---
        ax_mapped = ay
        ay_mapped = -ax
        az_mapped = az
        gx_mapped = gy
        gy_mapped = -gx
        gz_mapped = gz

        # --- Convertir giroscopio a rad/s ---
        gx_rad = np.radians(gx_mapped)
        gy_rad = np.radians(gy_mapped)
        gz_rad = np.radians(gz_mapped)

        # --- Calibración del IMU si no está calibrado ---
        if not self.imu.calibrated:
            if self.imu.calibrate(gx_rad, gy_rad, gz_rad, self.calibration_steps):
                gui_print(">>> CALIBRACIÓN IMU COMPLETADA <<<")
            return True  # Salir hasta que calibre

        # --- Actualizar IMU con datos mapeados ---
        self.imu.update(ax_mapped, ay_mapped, az_mapped, gx_rad, gy_rad, gz_rad, dt)

        # --- PROCESAMIENTO DE FLEXION (mantener igual que main) ---
        analog_data = np.array(analogs_raw)
        if self.num_fingers_model == 5:
            analog_data = np.append(analog_data, analog_data[3]) 

        self.flex_min = np.minimum(self.flex_min, analog_data)
        self.flex_max = np.maximum(self.flex_max, analog_data)
        span = np.maximum(self.flex_max - self.flex_min, 1e-3)
        
        flex_norm = np.clip((analog_data - self.flex_min) / span, 0.0, 1.0)
        target_positions_rad = MIN_FLEXION_RAD + (MAX_FLEXION_RAD - MIN_FLEXION_RAD) * (1.0 - flex_norm)

        # --- ACTUALIZAR POSICIÓN Y ORIENTACIÓN EN PYBULLET (de pruebas.py) ---
        self.base_target_pos = np.array(self.base_target_pos) + np.array(self.translation_vector)
        
        # Obtener cuaternión relativo [x, y, z, w]
        raw_orn = self.imu.get_pybullet_quaternion()
        
        # Corrección de pitch para que la mano esté plana (90 grados en Y)
        correction_euler = [0, 1.57, 0] 
        correction_quat = p.getQuaternionFromEuler(correction_euler)
        
        _, target_orn = p.multiplyTransforms([0,0,0], raw_orn, [0,0,0], correction_quat)
        
        curr_pos, curr_orn = p.getBasePositionAndOrientation(self.hand_id)
        
        # Suavizado dinámico basado en velocidad de rotación
        rot_speed = np.linalg.norm([gx_rad, gy_rad, gz_rad])
        current_smooth = 0.7 if rot_speed > 2.0 else self.smoothing_factor
        
        new_pos = np.array(curr_pos) * (1 - self.smoothing_factor) + self.base_target_pos * self.smoothing_factor
        new_orn = p.getQuaternionSlerp(curr_orn, target_orn, current_smooth)
        
        p.resetBasePositionAndOrientation(self.hand_id, new_pos, new_orn)
        self.base_target_pos = new_pos 

        # Actualizar joints de dedos
        joint_updates_indices = []
        joint_updates_pos = []
        
        for i, (finger_key, joint_names) in enumerate(self.finger_mapping.items()):
            target_val = target_positions_rad[i]
            for j, joint_name in enumerate(joint_names):
                if joint_name in self.joint_indices:
                    val = np.clip(target_val, MIN_FLEXION_RAD, MAX_FLEXION_RAD)
                    joint_updates_indices.append(self.joint_indices[joint_name])
                    joint_updates_pos.append(val)
        
        if joint_updates_indices:
            p.setJointMotorControlArray(self.hand_id, joint_updates_indices, p.POSITION_CONTROL, 
                                        targetPositions=joint_updates_pos, 
                                        forces=[500]*len(joint_updates_pos))
        return True

    def visualization_loop(self):
        """Bucle principal de visualización y simulación en PyBullet."""
        if self.setup_pybullet() == -1: 
            return

        gui_print("\n" + "="*60)
        gui_print("SIMULADOR DE MANO HÁPTICA")
        gui_print("="*60)
        gui_print("\n--- CONTROLES ---")
        gui_print("Ratón: Rotar cámara (Ctrl+Click), Zoom (Rueda), Pan (Shift+Click)")
        gui_print("Click izquierdo en zona de mano: Simula toque para haptic")
        gui_print("Teclas 1-6: Mover mano en XYZ")
        gui_print("Tecla R: Recalibrar sensores flex (abrir/cerrar mano)")
        gui_print("Tecla O: Resetear orientación (poner mano plana)")
        gui_print("\n--- COMANDOS CONSOLA ---")
        gui_print("Comandos Arduino: P0-P255, M0-M20, S, T, B, STOP, ?")
        gui_print("B, BCnn, BPxx, BOxx -> burst")
        gui_print("T, Fxx, Wxx, TDxx -> train")
        gui_print("'config <zona> <M> <tipo>': Reconfigurar feedback")
        gui_print("  Ejemplo: config THUMB_PROX 5 S")
        gui_print("'?': Ver ayuda Arduino")
        gui_print("'quit': Salir")
        gui_print("="*60 + "\n")

        collision_check_counter = 0
        while running:
            # Procesar datos de sensores
            if data_queue:
                self.process_sensor_data(data_queue[-1]) 
                data_queue.clear() 

            self.handle_keyboard_input()
            
            # Chequear colisiones con cubo cada 5 frames
            collision_check_counter += 1
            if collision_check_counter >= 5:
                self.check_collisions()
                collision_check_counter = 0


            p.stepSimulation()
            time.sleep(1./240.)

# --- SERIAL LOOP ---
def serial_loop():
    global running
    gui_print(f"[SERIAL] Conectando a {SERIAL_PORT}...")
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2)  # Esperar reset del Arduino
        gui_print(f"[SERIAL] ✓ Conectado a {SERIAL_PORT}\n")
        
        while running:
            # Enviar comandos pendientes
            while command_queue:
                cmd = command_queue.popleft()
                ser.write((cmd + '\n').encode('utf-8'))
            
            # Leer respuestas
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        # Distinguir datos de sensores vs respuestas
                        if "," in line and (line[0].isdigit() or line[0] == '-'):
                            # Formato antiguo sin prefijo
                            data_queue.append(line)
                        else:
                            # Es respuesta de comando
                            gui_print(f"[ARDUINO] {line}")
            except Exception as e:
                gui_print(f"[SERIAL] Error: {e}")
                
    except serial.SerialException as e:
        gui_print(f"[SERIAL] ✗ No se pudo abrir {SERIAL_PORT}: {e}")
        running = False
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            gui_print("[SERIAL] Desconectado")

if __name__ == "__main__":
    # Iniciar thread serial
    t_serial = threading.Thread(target=serial_loop, daemon=True)
    t_serial.start()
    
    sim = HandSimulator()
    
    # Configurar GUI Tkinter para consola serial y comandos
    root = tk.Tk()
    root.title("Haptic Hand Simulator Console")
    
    text = tk.Text(root, height=20, width=80)
    text.pack()
    
    entry = tk.Entry(root, width=80)
    entry.pack()
    
    def send_command(event):
        cmd = entry.get().strip()
        if not cmd:
            return
        entry.delete(0, tk.END)
        gui_print(f"> {cmd}")
        
        cmd_lower = cmd.lower()
        if cmd_lower == "quit":
            global running
            running = False
            root.quit()
            return
        
        if cmd_lower.startswith("config "):
            parts = cmd.split()
            if len(parts) >= 4:
                zone = parts[1].upper()
                try:
                    m_pos = int(parts[2])
                    stim_type = parts[3].upper()
                    sim.haptic_config.configure_zone(zone, m_pos, stim_type)
                except ValueError:
                    gui_print("[CONFIG] Formato inválido")
            else:
                gui_print("[CONFIG] Formato: config <zona> <M> <tipo>")
        else:
            command_queue.append(cmd.upper())
    
    entry.bind("<Return>", send_command)
    
    def update_gui():
        try:
            while not gui_queue.empty():
                typ, msg = gui_queue.get_nowait()
                if typ == "output":
                    text.insert(tk.END, msg + "\n")
                    text.see(tk.END)
        except:
            pass
        if running:
            root.after(100, update_gui)
    
    update_gui()
    
    # Iniciar thread de visualización PyBullet
    t_vis = threading.Thread(target=sim.visualization_loop, daemon=True)
    t_vis.start()
    
    root.mainloop()
    
    running = False
    time.sleep(0.5)
    if p.isConnected():
        p.disconnect()