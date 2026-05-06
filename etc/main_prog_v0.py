import threading
import numpy as np
import pybullet as p
import pybullet_data
import time
import serial  # Requiere: pip install pyserial
from collections import deque

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

# Cola de datos
data_queue = deque(maxlen=1)
command_queue = deque()
running = True

class HapticConfig:
    """Configuración de feedback háptico para cada zona de colisión"""
    def __init__(self):
        # Mapeo: zona -> (posición M, tipo de estímulo)
        # Tipos: 'S' = single, 'T' = train, 'B' = burst
        self.zone_haptic_map = {
            # Mano de 5 dedos
            'PALM': {'M': 19, 'stim_type': 'B', 'stim_params': {}},
            'THUMB': {'M': 1, 'stim_type': 'T', 'stim_params': {}},
            'INDEX': {'M': 3, 'stim_type': 'T', 'stim_params': {}},
            'MIDDLE': {'M': 5, 'stim_type': 'T', 'stim_params': {}},
            'RING': {'M': 7, 'stim_type': 'T', 'stim_params': {}},
            'PINKY': {'M': 9, 'stim_type': 'T', 'stim_params': {}},
            
            # Mano de 4 dedos (alternativo)
            'D0': {'M': 1, 'stim_type': 'T', 'stim_params': {}},
            'D1': {'M': 3, 'stim_type': 'T', 'stim_params': {}},
            'D2': {'M': 5, 'stim_type': 'T', 'stim_params': {}},
            'D3': {'M': 7, 'stim_type': 'T', 'stim_params': {}},
        }
    
    def get_haptic_commands(self, zone):
        """Retorna los comandos para activar el feedback háptico de una zona"""
        if zone not in self.zone_haptic_map:
            return []
        
        config = self.zone_haptic_map[zone]
        commands = []
        
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
        """Permite reconfigurar dinámicamente una zona"""
        if zone in self.zone_haptic_map:
            self.zone_haptic_map[zone]['M'] = m_pos
            self.zone_haptic_map[zone]['stim_type'] = stim_type
            self.zone_haptic_map[zone]['stim_params'] = stim_params
            print(f"[CONFIG] Zona {zone} → M{m_pos}, Estímulo: {stim_type}")

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
        
        # --- VARIABLES DE FILTRADO ---
        self.yaw_f = 0.0
        self.ax_prev = self.ay_prev = self.az_prev = 0.0
        self.gx_prev = self.gy_prev = self.gz_prev = 0.0
        self.alpha_s = 0.5      
        self.yaw_alpha = 0.5    
        self.smoothing_factor = 0.3 
        
        self.base_target_pos = np.array([0.0, 0.0, 0.3]) 
        self.base_target_orn = p.getQuaternionFromEuler([0, 0, 0])
        self.translation_vector = [0.0, 0.0, 0.0]
        self.beta_angle = 0.0
        self.last_time = time.time()
        self.yaw_offset = 0.0 

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
        self.four_finger_joints = {
            'FINGER_0': ['joint_1.0', 'joint_2.0', 'joint_3.0'],
            'FINGER_1': ['joint_5.0', 'joint_6.0', 'joint_7.0'],
            'FINGER_2': ['joint_9.0', 'joint_10.0', 'joint_11.0'],
            'FINGER_3': ['joint_13.0', 'joint_14.0', 'joint_15.0'],
        }
        self.five_finger_joints = {
            'PINKY': ['right_hand_Pinky', 'right_hand_j13', 'right_hand_j17'],
            'THUMB_INDEX': ['right_hand_Thumb_Opposition', 'right_hand_Thumb_Flexion', 'right_hand_j3', 'right_hand_j4',
                            'right_hand_Index_Finger_Proximal', 'right_hand_Index_Finger_Distal', 'right_hand_j14'],
            'MIDDLE': ['right_hand_Middle_Finger_Proximal', 'right_hand_Middle_Finger_Distal', 'right_hand_j15'],
            'RING': ['right_hand_Ring_Finger', 'right_hand_j12', 'right_hand_j16'],
        }
        
        self.four_collision_zones = {
            'PALM': [-1], 
            'D0': ['link_3.0'], 
            'D1': ['link_7.0'], 
            'D2': ['link_11.0'], 
            'D3': ['link_15.0']
        }
        self.five_collision_zones = {
            'PALM': [-1], 
            'THUMB': ['right_hand_c'], 
            'INDEX': ['right_hand_t'], 
            'MIDDLE': ['right_hand_s'], 
            'RING': ['right_hand_r'], 
            'PINKY': ['right_hand_q']
        }

    def setup_pybullet(self):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)  # Gravedad para que objetos caigan
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
                                     self.base_target_orn, useFixedBase=False)
        except:
            print("Error cargando URDF.")
            return -1

        # Indexar joints y links
        num_joints = p.getNumJoints(self.hand_id)
        for i in range(num_joints):
            info = p.getJointInfo(self.hand_id, i)
            self.joint_indices[info[1].decode('utf-8')] = i
            self.link_indices[info[12].decode('utf-8')] = i

        # Detectar tipo de mano (4 o 5 dedos)
        if 'right_hand_Thumb_Opposition' in self.joint_indices:
            self.finger_mapping = self.five_finger_joints
            self.num_fingers_model = 5
            self.collision_zones = {k: [self.link_indices.get(l) for l in v if l in self.link_indices] 
                                   for k, v in self.five_collision_zones.items()}
        else:
            self.finger_mapping = self.four_finger_joints
            self.num_fingers_model = 4
            self.collision_zones = {k: [self.link_indices.get(l) for l in v if l in self.link_indices] 
                                   for k, v in self.four_collision_zones.items()}

        self.last_collision_state = {k: False for k in self.collision_zones}

        self.reset_calibration()

        # Control de motores
        joint_uids = list(self.joint_indices.values())
        p.setJointMotorControlArray(self.hand_id, joint_uids, p.POSITION_CONTROL, 
                                    forces=[500]*len(joint_uids))
        
        # Inicializar potenciómetro al máximo
        command_queue.append("P255")
        
        print(f"\n[SETUP] Mano cargada: {self.num_fingers_model} dedos")
        print(f"[SETUP] Zonas de colisión: {list(self.collision_zones.keys())}")
        print(f"[SETUP] Cubo fijo en {CUBE_POSITION}, tamaño: {CUBE_SIZE}m\n")
        
        return self.hand_id

    def reset_calibration(self):
        print("[INFO] Recalibrando sensores...")
        self.flex_min = np.full(self.num_fingers_model, np.inf)
        self.flex_max = np.full(self.num_fingers_model, -np.inf)

    def reset_orientation(self):
        print("[INFO] Reseteando orientación (Zero Yaw)...")
        self.yaw_offset = self.beta_angle

    def handle_keyboard_input(self):
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
        """Detecta colisiones y envía feedback háptico"""
        contact_points = p.getContactPoints(self.hand_id, self.cube_id)
        
        # Cambiar color del cubo según si hay contacto
        if contact_points:
            p.changeVisualShape(self.cube_id, -1, rgbaColor=[1.0, 0.3, 0.0, 1.0])
        else:
            p.changeVisualShape(self.cube_id, -1, rgbaColor=[0.2, 0.6, 0.8, 1])
        
        # Detectar qué zonas están tocando
        touched_zones = {k: False for k in self.collision_zones}
        for cp in contact_points:
            link_index = cp[3]  # Link de la mano que colisiona
            for zone, links in self.collision_zones.items():
                if link_index in links:
                    touched_zones[zone] = True
                    break
        
        # Procesar cambios de estado de colisión
        for zone in self.collision_zones:
            # INICIO DE COLISIÓN
            if touched_zones[zone] and not self.last_collision_state[zone]:
                print(f"\n[COLLISION]  Zona {zone} tocando cubo")
                
                # Enviar comandos de activación
                haptic_commands = self.haptic_config.get_haptic_commands(zone)
                for cmd in haptic_commands:
                    command_queue.append(cmd)
                    print(f"[HAPTIC] → {cmd}")
                
                self.active_haptic_zones.add(zone)
            
            # FIN DE COLISIÓN
            elif not touched_zones[zone] and self.last_collision_state[zone]:
                print(f"\n[COLLISION] ✗ Zona {zone} dejó de tocar")
                
                # Detener estímulo
                stop_commands = self.haptic_config.get_stop_commands(zone)
                for cmd in stop_commands:
                    command_queue.append(cmd)
                    print(f"[HAPTIC] → {cmd}")
                
                self.active_haptic_zones.discard(zone)
        
        self.last_collision_state = touched_zones

    def process_sensor_data(self, line):
        """Procesa datos del Arduino: time, A3, A2, A1, A0, ax, ay, az, gx, gy, gz"""
        parts = line.split(",")
        if len(parts) < 11: 
            return False
        
        try:
            _, *vals = [float(v) for v in parts[:11]]
            
            # Mapeo de analógicos: [A0, A3, A2, A1]
            analogs_raw = [vals[3], vals[0], vals[1], vals[2]]
            
            # IMU
            ax, ay, az, gx, gy, gz = vals[4:10]
        except ValueError: 
            return False

        dt = time.time() - self.last_time
        self.last_time = time.time()

        # 1. IMU SMOOTHING
        ax_, ay_ = ay, -ax
        gx_, gy_ = gy, -gx
        
        self.ax_prev = self.ax_prev * (1 - self.alpha_s) + ax_ * self.alpha_s
        self.ay_prev = self.ay_prev * (1 - self.alpha_s) + ay_ * self.alpha_s
        self.az_prev = self.az_prev * (1 - self.alpha_s) + az * self.alpha_s
        
        gx_rad, gy_rad, gz_rad = np.radians([gx_, gy_, gz])
        self.beta_angle += gz_rad * dt 
        
        effective_yaw = self.beta_angle - self.yaw_offset
        self.yaw_f = self.yaw_f * (1 - self.yaw_alpha) + effective_yaw * self.yaw_alpha
        
        roll = np.arctan2(self.ay_prev, self.az_prev)
        pitch = np.arctan2(-self.ax_prev, np.sqrt(self.ay_prev**2 + self.az_prev**2))

        # 2. FLEXION
        analog_data = np.array(analogs_raw)
        if self.num_fingers_model == 5:
            analog_data = np.append(analog_data, analog_data[3]) 

        self.flex_min = np.minimum(self.flex_min, analog_data)
        self.flex_max = np.maximum(self.flex_max, analog_data)
        span = np.maximum(self.flex_max - self.flex_min, 1e-3)
        
        flex_norm = np.clip((analog_data - self.flex_min) / span, 0.0, 1.0)
        target_positions_rad = MIN_FLEXION_RAD + (MAX_FLEXION_RAD - MIN_FLEXION_RAD) * (1.0 - flex_norm)

        # 3. ACTUALIZAR PYBULLET
        self.base_target_pos = np.array(self.base_target_pos) + np.array(self.translation_vector)
        target_orn = p.getQuaternionFromEuler([roll, pitch, self.yaw_f])
        
        curr_pos, curr_orn = p.getBasePositionAndOrientation(self.hand_id)
        
        new_pos = np.array(curr_pos) * (1 - self.smoothing_factor) + self.base_target_pos * self.smoothing_factor
        new_orn = p.getQuaternionSlerp(curr_orn, target_orn, self.smoothing_factor)
        
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
        if self.setup_pybullet() == -1: 
            return

        print("\n" + "="*60)
        print("SIMULADOR DE MANO HÁPTICA")
        print("="*60)
        print("\n--- CONTROLES ---")
        print("Ratón: Rotar cámara (Ctrl+Click), Zoom (Rueda), Pan (Shift+Click)")
        print("Teclas 1-6: Mover mano en XYZ")
        print("Tecla R: Recalibrar sensores flex (abrir/cerrar mano)")
        print("Tecla O: Resetear orientación (poner mano plana)")
        print("\n--- COMANDOS CONSOLA ---")
        print("Comandos Arduino: P0-P255, M0-M20, S, T, B, STOP, etc.")
        print("'config <zona> <M> <tipo>': Reconfigurar feedback")
        print("  Ejemplo: config THUMB 5 S")
        print("'?': Ver ayuda Arduino")
        print("'quit': Salir")
        print("="*60 + "\n")

        collision_check_counter = 0
        while running:
            # Procesar datos de sensores
            if data_queue:
                self.process_sensor_data(data_queue[-1]) 
                data_queue.clear() 

            self.handle_keyboard_input()
            
            # Chequear colisiones cada 5 frames (optimización)
            collision_check_counter += 1
            if collision_check_counter >= 5:
                self.check_collisions()
                collision_check_counter = 0

            p.stepSimulation()
            time.sleep(1./240.)

# --- SERIAL LOOP ---
def serial_loop():
    global running
    print(f"[SERIAL] Conectando a {SERIAL_PORT}...")
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2)  # Esperar reset del Arduino
        print(f"[SERIAL] ✓ Conectado a {SERIAL_PORT}\n")
        
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
                            print(f"[ARDUINO] {line}")
            except Exception as e:
                print(f"[SERIAL] Error: {e}")
                
    except serial.SerialException as e:
        print(f"[SERIAL] ✗ No se pudo abrir {SERIAL_PORT}: {e}")
        running = False
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("[SERIAL] Desconectado")

# --- COMMAND INPUT LOOP ---
def command_input_loop():
    global running
    sim = None
    
    while running:
        try:
            cmd = input("").strip()
            if not cmd:
                continue
                
            if cmd.lower() == "quit":
                running = False
                break
            
            # Comando especial: reconfigurar zona
            if cmd.lower().startswith("config "):
                parts = cmd.split()
                if len(parts) >= 4:
                    zone = parts[1].upper()
                    try:
                        m_pos = int(parts[2])
                        stim_type = parts[3].upper()
                        if sim and sim.haptic_config:
                            sim.haptic_config.configure_zone(zone, m_pos, stim_type)
                        else:
                            print("[CONFIG] Simulador no inicializado aún")
                    except ValueError:
                        print("[CONFIG] Formato: config <zona> <M> <tipo>")
                else:
                    print("[CONFIG] Formato: config <zona> <M> <tipo>")
                    print("           Ejemplo: config THUMB 5 S")
            else:
                # Comando normal para Arduino
                command_queue.append(cmd.upper())
                
        except EOFError:
            pass
        except Exception as e:
            print(f"[INPUT] Error: {e}")

if __name__ == "__main__":
    # Iniciar threads
    t_serial = threading.Thread(target=serial_loop, daemon=True)
    t_serial.start()
    
    t_input = threading.Thread(target=command_input_loop, daemon=True)
    t_input.start()
    
    # Simulador en el thread principal
    sim = HandSimulator()
    
    # Hacer el simulador accesible al thread de comandos
    import __main__
    __main__.sim = sim
    
    try:
        sim.visualization_loop()
    except KeyboardInterrupt:
        print("\n[MAIN] Interrupción recibida")
    finally:
        running = False
        time.sleep(0.5)
        p.disconnect()
        print("[MAIN] Simulador cerrado")