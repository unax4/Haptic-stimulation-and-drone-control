"""
╔══════════════════════════════════════════════════════════════════════════╗
║         FRONTÓN HÁPTICO  —  PyBullet + Guante Arduino RP2040            ║
╠══════════════════════════════════════════════════════════════════════════╣
║  MECÁNICA:                                                               ║
║   • Guante se mueve SOLO en eje X por inclinación lateral (roll IMU)    ║
║   • Tras rebote en pared, pelota calculada cinemáticamente hacia guante  ║
║   • Orientación + flexión dedos: copiado literal de main_prog_vMahony   ║
║   • Cancha compacta para que siempre sea alcanzable                      ║
║                                                                          ║
║  CONTROLES PyBullet:                                                     ║
║   ESPACIO  → Saque                                                       ║
║   O        → Recentrar orientación IMU                                   ║
║   R        → Recalibrar sensores flex                                    ║
║                                                                          ║
║  DATOS ARDUINO: timestamp, A3, A2, A1, A0, ax, ay, az, gx, gy, gz      ║
╚══════════════════════════════════════════════════════════════════════════╝
"""

import threading
import numpy as np
import pybullet as p
import pybullet_data
import time
import serial
from collections import deque
import tkinter as tk
from queue import Queue
import math

# ══════════════════════════════════════════════════════════════════
#  CONFIG SERIAL
# ══════════════════════════════════════════════════════════════════
SERIAL_PORT = "COM3"
BAUD_RATE   = 115200

# ══════════════════════════════════════════════════════════════════
#  CONSTANTES  (copiadas del original donde aplica)
# ══════════════════════════════════════════════════════════════════
MIN_FLEXION_RAD = 0.0
MAX_FLEXION_RAD = 2.0

# ── Cancha COMPACTA ───────────────────────────────────────────────
#   Todo está escalado para que la pelota sea fácil de golpear.
#   La cámara ve todo desde un ángulo lateral.
WALL_Y      = 2.2    # distancia a la pared (m) — antes 5.5, ahora 3.5
WALL_W      = 2.6    # ancho de la pared
WALL_H      = 2.2    # alto de la pared

GLOVE_Y     = 0.0    # profundidad fija del guante (origen de profundidad)
GLOVE_Z     = 1.05   # altura fija del guante
GLOVE_X_MAX = 0.9    # límite lateral ±

# ── Pelota ────────────────────────────────────────────────────────
BALL_R      = 0.033  # radio (m)
BALL_MASS   = 0.058  # kg

# ── Restituciones ─────────────────────────────────────────────────
REST_GLOVE      = 0.80
REST_WALL       = 0.78
REST_SIDE       = 0.60
REST_FLOOR      = 0.50
REST_CEIL       = 0.45

# ── Física ────────────────────────────────────────────────────────
GRAVITY     = -9.81
SIM_HZ      = 240
DT          = 1.0 / SIM_HZ

# ── Velocidad de saque ────────────────────────────────────────────
SERVE_VY    = 7.0    # m/s hacia la pared en el saque

# ══════════════════════════════════════════════════════════════════
#  ESTADO COMPARTIDO
# ══════════════════════════════════════════════════════════════════
data_queue    = deque(maxlen=1)
command_queue = deque()
gui_queue     = Queue()
running       = True

# Sensibilidad de inclinación — modificable desde el slider GUI
g_sensitivity = 3.0   # (m/s de desplazamiento lateral) / radián de roll

def gui_print(msg):
    gui_queue.put(("output", msg))


# ══════════════════════════════════════════════════════════════════
#  HAPTIC CONFIG  —  copiado literal del original
# ══════════════════════════════════════════════════════════════════
class HapticConfig:
    def __init__(self):
        self.zone_haptic_map = {
            'PALM': [
                {'M': 19, 'stim_type': 'B', 'stim_params': {}},
            ],
            'THUMB': [
                {'M': 1, 'stim_type': 'T', 'stim_params': {}},
                {'M': 2, 'stim_type': 'B', 'stim_params': {}}
            ],
            'INDEX_BOTTOM': [
                {'M': 12, 'stim_type': 'T', 'stim_params': {}}
            ],
            'INDEX_TOP': [
                {'M': 3, 'stim_type': 'T', 'stim_params': {}},
            ],
            'MIDDLE_BOTTOM': [
                {'M': 14, 'stim_type': 'T', 'stim_params': {}}
            ],
            'MIDDLE_TOP': [
                {'M': 5, 'stim_type': 'T', 'stim_params': {}},
            ],
            'RING_BOTTOM': [
                {'M': 16, 'stim_type': 'T', 'stim_params': {}}
            ],
            'RING_TOP': [
                {'M': 7, 'stim_type': 'T', 'stim_params': {}},
            ],
            'PINKY_BOTTOM': [
                {'M': 18, 'stim_type': 'T', 'stim_params': {}}
            ],
            'PINKY_TOP': [
                {'M': 9, 'stim_type': 'T', 'stim_params': {}},
            ],
        }

    def get_haptic_commands(self, zone):
        if zone not in self.zone_haptic_map:
            return []
        configs = self.zone_haptic_map[zone]
        commands = []
        for config in configs:
            commands.append(f"M{config['M']}")
            stim_type = config['stim_type']
            if stim_type == 'S':
                commands.append('S')
            elif stim_type == 'T':
                commands.append('T')
            elif stim_type == 'B':
                commands.append('B')
        return commands

    def get_stop_commands(self, zone):
        return ['STOP']

    def configure_zone(self, zone, m_pos, stim_type='T', **stim_params):
        if zone in self.zone_haptic_map:
            self.zone_haptic_map[zone][0]['M'] = m_pos
            self.zone_haptic_map[zone][0]['stim_type'] = stim_type
            self.zone_haptic_map[zone][0]['stim_params'] = stim_params
            gui_print(f"[CONFIG] Zona {zone} → M{m_pos}, Estímulo: {stim_type}")


# ══════════════════════════════════════════════════════════════════
#  IMU MAHONY  —  copiado literal del original
# ══════════════════════════════════════════════════════════════════
class IMUSensorFusion:
    def __init__(self):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.Kp = 5.0
        self.Ki = 0.02
        self.eInt = np.array([0.0, 0.0, 0.0])
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.calibrated = False
        self.bias_samples = []
        self.q_offset = np.array([1.0, 0.0, 0.0, 0.0])
        self.forward_offset = None

    def calibrate(self, gx, gy, gz, max_samples=100):
        self.bias_samples.append([gx, gy, gz])
        if len(self.bias_samples) >= max_samples:
            self.gyro_bias = np.mean(self.bias_samples, axis=0)
            self.calibrated = True
            self.reset_orientation()
            gui_print(f"[IMU] Calibrado. Bias detectado: {self.gyro_bias}")
            return True
        return False

    def reset_orientation(self):
        self.forward_offset = self.q.copy()
        self.q_offset = np.array([self.q[0], -self.q[1], -self.q[2], -self.q[3]])
        gui_print(">>> ORIENTACIÓN RECENTRADA (Tecla O) <<<")

    def update(self, ax, ay, az, gx, gy, gz, dt):
        if self.calibrated:
            gx -= self.gyro_bias[0]
            gy -= self.gyro_bias[1]
            gz -= self.gyro_bias[2]

        q = self.q
        norm_a = np.sqrt(ax*ax + ay*ay + az*az)
        if norm_a == 0.0:
            return
        ax /= norm_a; ay /= norm_a; az /= norm_a

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

        pa, pb, pc = q[0], q[1], q[2]
        q[0] += (-q[1] * gx - q[2] * gy - q[3] * gz) * (0.5 * dt)
        q[1] += (pa * gx + q[2] * gz - q[3] * gy) * (0.5 * dt)
        q[2] += (pa * gy - pb * gz + q[3] * gx) * (0.5 * dt)
        q[3] += (pa * gz + pb * gy - pc * gx) * (0.5 * dt)

        self.q = q / np.linalg.norm(q)
        return self.q

    def get_pybullet_quaternion(self):
        qo = self.q_offset
        qa = self.q
        res = np.array([
            qo[0]*qa[0] - qo[1]*qa[1] - qo[2]*qa[2] - qo[3]*qa[3],
            qo[0]*qa[1] + qo[1]*qa[0] + qo[2]*qa[3] - qo[3]*qa[2],
            qo[0]*qa[2] - qo[1]*qa[3] + qo[2]*qa[0] + qo[3]*qa[1],
            qo[0]*qa[3] + qo[1]*qa[2] - qo[2]*qa[1] + qo[3]*qa[0]
        ])
        return [res[1], res[2], res[3], res[0]]

    def get_roll_rad(self):
        """Roll relativo al offset = inclinación lateral del guante."""
        qo = self.q_offset
        qa = self.q
        # Cuaternión relativo [w, x, y, z]
        w = qo[0]*qa[0] - qo[1]*qa[1] - qo[2]*qa[2] - qo[3]*qa[3]
        x = qo[0]*qa[1] + qo[1]*qa[0] + qo[2]*qa[3] - qo[3]*qa[2]
        y = qo[0]*qa[2] - qo[1]*qa[3] + qo[2]*qa[0] + qo[3]*qa[1]
        z = qo[0]*qa[3] + qo[1]*qa[2] - qo[2]*qa[1] + qo[3]*qa[0]
        sinr = 2.0 * (w*x + y*z)
        cosr = 1.0 - 2.0 * (x*x + y*y)
        return math.atan2(sinr, cosr)


# ══════════════════════════════════════════════════════════════════
#  JUEGO PRINCIPAL
# ══════════════════════════════════════════════════════════════════
class FrontonGame:
    def __init__(self, urdf_filename="schunk_svh_hand_right.urdf"):
        self.urdf_filename = urdf_filename

        # IDs PyBullet
        self.hand_id   = -1
        self.ball_id   = -1
        self.wall_id   = -1
        self.floor_id  = -1
        self.lwall_id  = -1
        self.rwall_id  = -1
        self.ceil_id   = -1

        # Hápticos e IMU
        self.haptic_config = HapticConfig()
        self.imu           = IMUSensorFusion()
        self.calibration_steps = 150

        # ── Variables copiadas del original ──────────────────────
        self.num_fingers_model = 0
        self.joint_indices     = {}
        self.link_indices      = {}
        self.finger_mapping    = {}
        self.collision_zones   = {}
        self.last_collision_state = {}
        self.active_haptic_zones  = set()

        self.base_target_pos   = np.array([0.0, GLOVE_Y, GLOVE_Z])
        self.smoothing_factor  = 0.3
        self.last_time         = time.time()

        self.flex_min = np.full(5, np.inf)
        self.flex_max = np.full(5, -np.inf)

        # ── Estado del juego ──────────────────────────────────────
        self.glove_x          = 0.0
        self.current_roll     = 0.0
        self.score            = 0
        self.ball_active      = False
        self.serve_ready      = True
        self.last_hit_time    = 0.0
        self.HIT_COOLDOWN     = 0.40
        self._wall_redirected = False   # flag: ¿ya redirigí este rebote?

        # HUD IDs
        self._hud_score = -1
        self._hud_msg   = -1
        self._hud_roll  = -1

        # Joints y zonas de colisión (copiados del original)
        self.five_finger_joints = {
            'PINKY': ['right_hand_Pinky', 'right_hand_j13', 'right_hand_j17'],
            'THUMB_INDEX': ['right_hand_Thumb_Opposition', 'right_hand_Thumb_Flexion',
                            'right_hand_j3', 'right_hand_j4',
                            'right_hand_Index_Finger_Proximal',
                            'right_hand_Index_Finger_Distal', 'right_hand_j14'],
            'MIDDLE': ['right_hand_Middle_Finger_Proximal',
                       'right_hand_Middle_Finger_Distal', 'right_hand_j15'],
            'RING': ['right_hand_Ring_Finger', 'right_hand_j12', 'right_hand_j16'],
        }
        self.five_collision_zones = {
            'PALM':         ['right_hand_base_link', 'right_hand_e1', 'right_hand_e2'],
            'THUMB':        ['right_hand_z', 'right_hand_a', 'right_hand_b', 'right_hand_c'],
            'INDEX_BOTTOM': ['right_hand_virtual_l', 'right_hand_l', 'right_hand_p'],
            'INDEX_TOP':    ['right_hand_t'],
            'MIDDLE_BOTTOM':['right_hand_virtual_k', 'right_hand_k', 'right_hand_o'],
            'MIDDLE_TOP':   ['right_hand_s'],
            'RING_BOTTOM':  ['right_hand_virtual_j', 'right_hand_j', 'right_hand_n'],
            'RING_TOP':     ['right_hand_r'],
            'PINKY_BOTTOM': ['right_hand_virtual_i', 'right_hand_i', 'right_hand_m'],
            'PINKY_TOP':    ['right_hand_q'],
        }

    # ──────────────────────────────────────────────────────────────
    #  SETUP PYBULLET
    # ──────────────────────────────────────────────────────────────
    def setup_pybullet(self):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, GRAVITY)
        p.setTimeStep(DT)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

        # Cámara lateral: ve toda la cancha
        p.resetDebugVisualizerCamera(
            cameraDistance=6.5,
            cameraYaw=90,
            cameraPitch=-18,
            cameraTargetPosition=[0.0, WALL_Y * 0.45, 1.3]
        )
        p.setPhysicsEngineParameter(
            numSolverIterations=150,
            numSubSteps=4)

        self._build_court()
        self._build_ball()
        self._build_hand()
        self._build_hud()

        gui_print("=" * 50)
        gui_print("  FRONTON HAPTICO — listo")
        gui_print("  ESPACIO → Saque")
        gui_print("  O       → Recentrar IMU")
        gui_print("  R       → Recalibrar flex")
        gui_print("=" * 50)

    # ──────────────────────────────────────────────────────────────
    #  CANCHA COMPACTA
    # ──────────────────────────────────────────────────────────────
    def _build_court(self):
        hw = WALL_W / 2.0
        depth = (WALL_Y - GLOVE_Y) + 1.5   # profundidad total del suelo

        # Suelo
        fc = p.createCollisionShape(p.GEOM_BOX,
                                    halfExtents=[hw, depth / 2, 0.04])
        fv = p.createVisualShape(p.GEOM_BOX,
                                 halfExtents=[hw, depth / 2, 0.04],
                                 rgbaColor=[0.20, 0.52, 0.20, 1.0])
        self.floor_id = p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=fc, baseVisualShapeIndex=fv,
            basePosition=[0.0, GLOVE_Y + depth / 2 - 0.5, 0.0])
        p.changeDynamics(self.floor_id, -1,
                         restitution=REST_FLOOR, lateralFriction=0.5)

        # Pared frontal (el frontón)
        wc = p.createCollisionShape(p.GEOM_BOX,
                                    halfExtents=[hw, 0.07, WALL_H / 2])
        wv = p.createVisualShape(p.GEOM_BOX,
                                 halfExtents=[hw, 0.07, WALL_H / 2],
                                 rgbaColor=[0.86, 0.79, 0.56, 1.0])
        self.wall_id = p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=wc, baseVisualShapeIndex=wv,
            basePosition=[0.0, WALL_Y, WALL_H / 2])
        p.changeDynamics(self.wall_id, -1,
                         restitution=REST_WALL, lateralFriction=0.10)

        # Paredes laterales
        sc = p.createCollisionShape(p.GEOM_BOX,
                                    halfExtents=[0.07, depth / 2, WALL_H / 2])
        for sign, attr in [(-1, 'lwall_id'), (1, 'rwall_id')]:
            sv = p.createVisualShape(p.GEOM_BOX,
                                     halfExtents=[0.07, depth / 2, WALL_H / 2],
                                     rgbaColor=[0.72, 0.58, 0.38, 0.65])
            wid = p.createMultiBody(
                baseMass=0, baseCollisionShapeIndex=sc, baseVisualShapeIndex=sv,
                basePosition=[sign * hw, GLOVE_Y + depth / 2 - 0.5, WALL_H / 2])
            setattr(self, attr, wid)
            p.changeDynamics(wid, -1, restitution=REST_SIDE, lateralFriction=0.18)

        # Techo
        cc = p.createCollisionShape(p.GEOM_BOX,
                                    halfExtents=[hw, depth / 2, 0.04])
        cv = p.createVisualShape(p.GEOM_BOX,
                                 halfExtents=[hw, depth / 2, 0.04],
                                 rgbaColor=[0.85, 0.85, 0.85, 0.15])
        self.ceil_id = p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=cc, baseVisualShapeIndex=cv,
            basePosition=[0.0, GLOVE_Y + depth / 2 - 0.5, WALL_H + 0.04])
        p.changeDynamics(self.ceil_id, -1, restitution=REST_CEIL, lateralFriction=0.10)

        # Línea blanca decorativa en el suelo (posición del jugador)
        lv = p.createVisualShape(p.GEOM_BOX,
                                 halfExtents=[hw, 0.025, 0.008],
                                 rgbaColor=[1.0, 1.0, 1.0, 0.7])
        p.createMultiBody(0, -1, lv, basePosition=[0.0, GLOVE_Y - 0.4, 0.01])

    # ──────────────────────────────────────────────────────────────
    #  PELOTA
    # ──────────────────────────────────────────────────────────────
    def _build_ball(self):
        bc = p.createCollisionShape(p.GEOM_SPHERE, radius=BALL_R)
        bv = p.createVisualShape(p.GEOM_SPHERE, radius=BALL_R,
                                 rgbaColor=[0.93, 0.97, 0.22, 1.0])
        self.ball_id = p.createMultiBody(
            baseMass=BALL_MASS,
            baseCollisionShapeIndex=bc,
            baseVisualShapeIndex=bv,
            basePosition=[0.0, GLOVE_Y + 0.3, GLOVE_Z + 0.05])
        p.changeDynamics(self.ball_id, -1,
            restitution=0.86,
            lateralFriction=0.15,
            linearDamping=0.0008,
            angularDamping=0.002,
            rollingFriction=0.0002,
            spinningFriction=0.0002)
        p.resetBaseVelocity(self.ball_id, [0, 0, 0], [0, 0, 0])

    # ──────────────────────────────────────────────────────────────
    #  GUANTE  —  setup copiado del original
    # ──────────────────────────────────────────────────────────────
    def _build_hand(self):
        init_pos = [self.glove_x, GLOVE_Y, GLOVE_Z]
        # El original usa [0,0,0] en la carga del URDF
        try:
            self.hand_id = p.loadURDF(
                self.urdf_filename, init_pos,
                p.getQuaternionFromEuler([0, 0, 0]),
                useFixedBase=False)
        except Exception as e:
            gui_print(f"[URDF] Error cargando: {e}")
            gui_print("[URDF] Usando raqueta simple.")
            self._make_simple_racket(init_pos)
            return

        # Masa 0 = kinematic (igual que el original: mano no cae)
        p.changeDynamics(self.hand_id, -1,
                         mass=0,
                         restitution=REST_GLOVE,
                         lateralFriction=0.6)

        # Indexar joints y links  — copiado literal del original
        num_joints = p.getNumJoints(self.hand_id)
        for i in range(num_joints):
            info = p.getJointInfo(self.hand_id, i)
            self.joint_indices[info[1].decode('utf-8')] = i
            self.link_indices[info[12].decode('utf-8')] = i
            p.changeDynamics(self.hand_id, i,
                             restitution=REST_GLOVE, lateralFriction=0.6)

        self.finger_mapping    = self.five_finger_joints
        self.num_fingers_model = 5
        self.collision_zones   = {
            k: [self.link_indices.get(l, -1) for l in v]
            for k, v in self.five_collision_zones.items()
        }
        self.last_collision_state = {k: False for k in self.collision_zones}
        self.reset_calibration()

        # Control de motores  — copiado del original
        joint_uids = list(self.joint_indices.values())
        p.setJointMotorControlArray(self.hand_id, joint_uids, p.POSITION_CONTROL,
                                    forces=[500] * len(joint_uids))
        command_queue.append("P255")

        gui_print(f"[SETUP] Mano cargada: {self.num_fingers_model} dedos")
        gui_print(f"[SETUP] Zonas colision: {list(self.collision_zones.keys())}")

    def _make_simple_racket(self, pos):
        rc = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.16, 0.04, 0.22])
        rv = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.16, 0.04, 0.22],
                                 rgbaColor=[0.75, 0.35, 0.10, 1.0])
        self.hand_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=rc,
            baseVisualShapeIndex=rv,
            basePosition=pos)
        p.changeDynamics(self.hand_id, -1, mass=0,
                         restitution=REST_GLOVE, lateralFriction=0.6)

    # ──────────────────────────────────────────────────────────────
    #  HUD
    # ──────────────────────────────────────────────────────────────
    def _build_hud(self):
        self._hud_score = p.addUserDebugText(
            "PUNTOS: 0",
            [-WALL_W/2 + 0.1, WALL_Y * 0.5, WALL_H + 0.20],
            textColorRGB=[1.0, 1.0, 0.0], textSize=2.0)
        self._hud_msg = p.addUserDebugText(
            "ESPACIO para sacar",
            [0.0, WALL_Y * 0.5, WALL_H + 0.55],
            textColorRGB=[0.3, 1.0, 0.4], textSize=1.5)
        self._hud_roll = p.addUserDebugText(
            "Roll: 0.0  X: 0.00",
            [-WALL_W/2 + 0.1, WALL_Y * 0.5, WALL_H - 0.15],
            textColorRGB=[0.5, 0.8, 1.0], textSize=1.2)

    def _refresh_hud(self, msg=""):
        p.addUserDebugText(
            f"PUNTOS: {self.score}",
            [-WALL_W/2 + 0.1, WALL_Y * 0.5, WALL_H + 0.20],
            textColorRGB=[1.0, 1.0, 0.0], textSize=2.0,
            replaceItemUniqueId=self._hud_score)
        rd = math.degrees(self.current_roll)
        p.addUserDebugText(
            f"Roll:{rd:+.1f}  X:{self.glove_x:+.2f}m",
            [-WALL_W/2 + 0.1, WALL_Y * 0.5, WALL_H - 0.15],
            textColorRGB=[0.5, 0.8, 1.0], textSize=1.2,
            replaceItemUniqueId=self._hud_roll)
        if msg:
            p.addUserDebugText(
                msg,
                [0.0, WALL_Y * 0.5, WALL_H + 0.55],
                textColorRGB=[0.3, 1.0, 0.4], textSize=1.5,
                replaceItemUniqueId=self._hud_msg)

    # ──────────────────────────────────────────────────────────────
    #  RESET CALIBRACIÓN FLEX  —  copiado del original
    # ──────────────────────────────────────────────────────────────
    def reset_calibration(self):
        gui_print("[INFO] Recalibrando sensores flex...")
        self.flex_min = np.full(self.num_fingers_model, np.inf)
        self.flex_max = np.full(self.num_fingers_model, -np.inf)

    # ──────────────────────────────────────────────────────────────
    #  PROCESS SENSOR DATA  —  COPIADO LITERAL DEL ORIGINAL
    #  (solo cambia que base_target_pos.x se controla por roll,
    #   y base_target_pos.y/z son fijos GLOVE_Y / GLOVE_Z)
    # ──────────────────────────────────────────────────────────────
    def process_sensor_data(self, line):
        """
        Procesa datos del Arduino: time, A3, A2, A1, A0, ax, ay, az, gx, gy, gz.
        Lógica de orientación y flexión copiada literal de main_prog_vMahony.py.
        La única diferencia: posición Y y Z del guante son fijas; X viene del roll.
        """
        parts = line.split(",")
        if len(parts) < 11:
            return False

        try:
            _, *vals = [float(v) for v in parts[:11]]
            # Mapeo de analógicos: [A0, A3, A2, A1]
            analogs_raw = [vals[3], vals[0], vals[1], vals[2]]
            # IMU raw values
            ax, ay, az, gx, gy, gz = vals[4:10]
        except ValueError:
            return False

        dt = time.time() - self.last_time
        self.last_time = time.time()

        # --- Mapeo de ejes IMU (igual que en el original) ---
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

        # --- Calibración del IMU ---
        if not self.imu.calibrated:
            if self.imu.calibrate(gx_rad, gy_rad, gz_rad, self.calibration_steps):
                gui_print(">>> CALIBRACIÓN IMU COMPLETADA <<<")
            return True

        # --- Actualizar IMU ---
        self.imu.update(ax_mapped, ay_mapped, az_mapped, gx_rad, gy_rad, gz_rad, dt)

        # --- PROCESAMIENTO DE FLEXIÓN (copiado literal) ---
        analog_data = np.array(analogs_raw)
        if self.num_fingers_model == 5:
            analog_data = np.append(analog_data, analog_data[3])

        self.flex_min = np.minimum(self.flex_min, analog_data)
        self.flex_max = np.maximum(self.flex_max, analog_data)
        span = np.maximum(self.flex_max - self.flex_min, 1e-3)

        flex_norm = np.clip((analog_data - self.flex_min) / span, 0.0, 1.0)
        target_positions_rad = MIN_FLEXION_RAD + (MAX_FLEXION_RAD - MIN_FLEXION_RAD) * (1.0 - flex_norm)

        # --- POSICIÓN Y ORIENTACIÓN (adaptado del original) ---
        # X: actualizar con roll × sensibilidad × dt  (integrado como velocidad)
        roll_rad = self.imu.get_roll_rad()
        self.current_roll = roll_rad
        target_x = roll_rad * g_sensitivity
        self.glove_x += (target_x - self.glove_x) * 0.25

        # base_target_pos: X variable, Y y Z fijos
        self.base_target_pos = np.array([self.glove_x, GLOVE_Y, GLOVE_Z])
        if self.imu.forward_offset is not None:
            raw_orn = p.multiplyTransforms(
                [0,0,0],
                raw_orn,
                [0,0,0],
                p.getQuaternionFromEuler([0,0,math.pi])
            )[1]
        # Obtener cuaternión relativo [x, y, z, w]  — copiado del original
        raw_orn = self.imu.get_pybullet_quaternion()

        # Corrección de pitch para que la mano esté plana (90 grados en Y)  — copiado
        correction_euler = [0, 1.57, 0]
        correction_quat  = p.getQuaternionFromEuler(correction_euler)
        _, target_orn = p.multiplyTransforms([0, 0, 0], raw_orn,
                                              [0, 0, 0], correction_quat)

        curr_pos, curr_orn = p.getBasePositionAndOrientation(self.hand_id)

        # Suavizado dinámico basado en velocidad de rotación  — copiado
        rot_speed     = np.linalg.norm([gx_rad, gy_rad, gz_rad])
        current_smooth = 0.7 if rot_speed > 2.0 else self.smoothing_factor

        new_pos = (np.array(curr_pos) * (1 - self.smoothing_factor)
                   + self.base_target_pos * self.smoothing_factor)
        new_orn = p.getQuaternionSlerp(curr_orn, target_orn, current_smooth)

        p.resetBasePositionAndOrientation(self.hand_id, new_pos, new_orn)
        self.base_target_pos = new_pos  # retroalimentar posición suavizada

        # --- Joints de dedos  — copiado literal ---
        joint_updates_indices = []
        joint_updates_pos     = []

        for i, (finger_key, joint_names) in enumerate(self.finger_mapping.items()):
            target_val = target_positions_rad[i]
            for joint_name in joint_names:
                if joint_name in self.joint_indices:
                    val = np.clip(target_val, MIN_FLEXION_RAD, MAX_FLEXION_RAD)
                    joint_updates_indices.append(self.joint_indices[joint_name])
                    joint_updates_pos.append(val)

        if joint_updates_indices:
            p.setJointMotorControlArray(
                self.hand_id,
                joint_updates_indices,
                p.POSITION_CONTROL,
                targetPositions=joint_updates_pos,
                forces=[500] * len(joint_updates_pos))

        # Actualizar glove_x con la posición suavizada real
        self.glove_x = float(new_pos[0])
        return True

    # ──────────────────────────────────────────────────────────────
    #  COLISIONES HÁPTICAS  —  copiado del original (check_collisions)
    # ──────────────────────────────────────────────────────────────
    def check_haptic_collisions(self):
        """Detecta qué zonas de la mano tocan la pelota y activa hápticos."""
        contact_points = p.getContactPoints(self.hand_id, self.ball_id)

        touched_zones = {k: False for k in self.collision_zones}
        any_stop = False

        for cp in contact_points:
            link_index = cp[3]   # link de la mano
            for zone, links in self.collision_zones.items():
                if link_index in links:
                    touched_zones[zone] = True

        for zone in self.collision_zones:
            if touched_zones[zone] and not self.last_collision_state[zone]:
                haptic_commands = self.haptic_config.get_haptic_commands(zone)
                for cmd in haptic_commands:
                    command_queue.append(cmd)
                self.active_haptic_zones.add(zone)

            elif not touched_zones[zone] and self.last_collision_state[zone]:
                self.active_haptic_zones.discard(zone)
                any_stop = True

        if any_stop:
            command_queue.append('STOP')
            for zone in self.active_haptic_zones:
                for cmd in self.haptic_config.get_haptic_commands(zone):
                    command_queue.append(cmd)

        self.last_collision_state = touched_zones

    # ──────────────────────────────────────────────────────────────
    #  SAQUE
    # ──────────────────────────────────────────────────────────────
    def _serve(self):
        spawn = [self.glove_x, GLOVE_Y + 0.18, GLOVE_Z + 0.04]
        p.resetBasePositionAndOrientation(self.ball_id, spawn, [0, 0, 0, 1])
        # Pequeño arco hacia arriba para que el saque parezca natural
        vz0 = math.sqrt(2.0 * abs(GRAVITY) * 0.45)
        vx0 = float(np.random.uniform(-0.3, 0.3))
        p.resetBaseVelocity(self.ball_id, [vx0, SERVE_VY, vz0], [0, 0, 0])

        self.ball_active        = True
        self.serve_ready        = False
        self._wall_redirected   = False
        self.last_hit_time      = time.time()
        self._refresh_hud("En juego!")
        gui_print(f"[SAQUE] spawn={[round(v,2) for v in spawn]}  "
                  f"vy={SERVE_VY}  vz={vz0:.2f}")

    # ──────────────────────────────────────────────────────────────
    #  REDIRECCIÓN CINEMÁTICA TRAS REBOTE EN PARED
    #
    #  Objetivo: tras rebotar en la pared, la pelota debe llegar a
    #  (glove_x, GLOVE_Y, GLOVE_Z) exactamente.
    #
    #  Sistema de ecuaciones (tiro oblicuo, eje Y negativo):
    #    dy   = GLOVE_Y - by          [ < 0, pelota se aleja de la pared ]
    #    vy_r = -|bvy_actual|         [ negativo = hacia el jugador       ]
    #    t    = dy / vy_r             [ > 0                               ]
    #    vz_r = (GLOVE_Z - bz)/t  -  ½·g·t
    #    vx_r = (glove_x  - bx)/t
    #
    #  Se impone vy mínimo para garantizar que siempre llega.
    # ──────────────────────────────────────────────────────────────
    def _redirect_to_glove(self):
        ball_pos, _ = p.getBasePositionAndOrientation(self.ball_id)
        ball_vel, _ = p.getBaseVelocity(self.ball_id)
        bx, by, bz  = ball_pos
        _,  bvy, _  = ball_vel

        # Velocidad de retorno en Y: conservamos magnitud, invertimos sentido
        vy_r = -abs(bvy)
        if abs(vy_r) < 3.0:
            vy_r = -5.5    # mínimo garantizado para que llegue al jugador

        # Tiempo de vuelo: dy = GLOVE_Y - by  (negativo), vy_r < 0  →  t > 0
        dy = GLOVE_Y - by
        t  = dy / vy_r
        t  = max(t, 0.08)   # nunca < 80 ms

        # Velocidades necesarias para alcanzar exactamente (glove_x, GLOVE_Z)
        vz_r = (GLOVE_Z - bz) / t  -  0.5 * GRAVITY * t
        predicted_x = self.glove_x + self.current_roll * 0.15
        vx_r = (predicted_x - bx) / t

        p.resetBaseVelocity(self.ball_id, [vx_r, vy_r, vz_r], [0, 0, 0])

        gui_print(
            f"[PARED->GUANTE]  t={t:.2f}s  "
            f"v=({vx_r:.2f}, {vy_r:.2f}, {vz_r:.2f})  "
            f"dest=({self.glove_x:.2f}, {GLOVE_Y:.2f}, {GLOVE_Z:.2f})"
        )

    # ──────────────────────────────────────────────────────────────
    #  GOLPE DEL GUANTE → lanza hacia la pared
    # ──────────────────────────────────────────────────────────────
    def _handle_hit(self):
        now = time.time()
        if now - self.last_hit_time < self.HIT_COOLDOWN:
            return

        contacts = p.getContactPoints(self.ball_id, self.hand_id)
        if not contacts:
            return

        self.last_hit_time    = now
        self.score           += 1
        self._wall_redirected = False  # siguiente rebote en pared → redirección

        ball_vel, _ = p.getBaseVelocity(self.ball_id)
        bv = np.array(ball_vel, dtype=float)

        # Invertir componente Y y asegurar mínimo hacia la pared
        bv[1] = abs(bv[1])
        if bv[1] < 4.5:
            bv[1] = SERVE_VY * 0.88

        # Pequeña variación lateral para dinamismo
        bv[0] += float(np.random.uniform(-0.4, 0.4))

        p.resetBaseVelocity(self.ball_id, bv.tolist(), [0, 0, 0])

        # Flash visual en la pelota
        p.changeVisualShape(self.ball_id, -1, rgbaColor=[1.0, 0.38, 0.05, 1.0])
        threading.Timer(0.09, lambda: p.changeVisualShape(
            self.ball_id, -1, rgbaColor=[0.93, 0.97, 0.22, 1.0])).start()

        self._refresh_hud(f"GOLPE!  Puntos: {self.score}")
        gui_print(f"[GOLPE]  v_out={np.round(bv, 2)}  score={self.score}")

    # ──────────────────────────────────────────────────────────────
    #  PELOTA PERDIDA
    # ──────────────────────────────────────────────────────────────
    def _check_ball_lost(self):
        pos, _ = p.getBasePositionAndOrientation(self.ball_id)
        vel, _ = p.getBaseVelocity(self.ball_id)
        speed  = float(np.linalg.norm(vel))

        # Pasó al jugador / cayó al suelo y se detuvo / salió lateral
        lost = (pos[1] < GLOVE_Y - 2.0
                or (pos[2] < 0.08 and speed < 0.7)
                or abs(pos[0]) > WALL_W / 2 + 0.5)

        if lost:
            gui_print("[FALLO] Pelota perdida — ESPACIO para sacar")
            self._refresh_hud("ESPACIO para sacar")
            self.ball_active  = False
            self.serve_ready  = True
            # Aparcar pelota fuera de la vista
            p.resetBasePositionAndOrientation(
                self.ball_id, [0.0, WALL_Y + 2.0, -1.0], [0, 0, 0, 1])
            p.resetBaseVelocity(self.ball_id, [0, 0, 0], [0, 0, 0])
            p.changeVisualShape(self.ball_id, -1, rgbaColor=[0.93, 0.97, 0.22, 1.0])

    # ──────────────────────────────────────────────────────────────
    #  TECLADO
    # ──────────────────────────────────────────────────────────────
    def handle_keyboard_input(self):
        events = p.getKeyboardEvents()

        if ord(' ') in events and events[ord(' ')] & p.KEY_WAS_TRIGGERED:
            if self.serve_ready:
                self._serve()

        if ord('o') in events and events[ord('o')] & p.KEY_WAS_TRIGGERED:
            self.imu.reset_orientation()
            self.glove_x      = 0.0
            self.current_roll = 0.0
            self.base_target_pos = np.array([0.0, GLOVE_Y, GLOVE_Z])

        if ord('r') in events and events[ord('r')] & p.KEY_WAS_TRIGGERED:
            self.reset_calibration()

    # ──────────────────────────────────────────────────────────────
    #  BUCLE PRINCIPAL
    # ──────────────────────────────────────────────────────────────
    def run(self):
        self.setup_pybullet()

        collision_counter = 0
        frame             = 0

        while running:
            t0 = time.time()

            # 1. Sensores IMU + flex
            if data_queue:
                self.process_sensor_data(data_queue[-1])
                data_queue.clear()

            # 2. Teclado
            self.handle_keyboard_input()

            # 3. Paso de física
            p.stepSimulation()

            # 4. Lógica del juego (cada 2 frames ≈ 120 Hz)
            if frame % 2 == 0 and self.ball_active:

                # Rebote en pared → redirección garantizada al guante
                wall_cts = p.getContactPoints(self.ball_id, self.wall_id)
                if wall_cts and not self._wall_redirected:
                    self._wall_redirected = True
                    self._redirect_to_glove()
                    # Flash en la pared
                    p.changeVisualShape(self.wall_id, -1,
                                        rgbaColor=[1.0, 0.55, 0.10, 1.0])
                    threading.Timer(0.14, lambda: p.changeVisualShape(
                        self.wall_id, -1,
                        rgbaColor=[0.86, 0.79, 0.56, 1.0])).start()
                elif not wall_cts:
                    self._wall_redirected = False  # listo para el siguiente rebote

                # Golpe del guante
                self._handle_hit()

                # Pelota perdida
                self._check_ball_lost()

            # 5. Hápticos colisión (cada 4 frames)
            collision_counter += 1
            if collision_counter >= 4:
                if self.ball_active:
                    self.check_haptic_collisions()
                collision_counter = 0

            # 6. HUD (cada 15 frames ≈ 16 Hz)
            if frame % 15 == 0:
                self._refresh_hud()

            frame += 1
            elapsed = time.time() - t0
            time.sleep(max(0.0, DT - elapsed))


# ══════════════════════════════════════════════════════════════════
#  THREAD SERIAL  —  copiado del original
# ══════════════════════════════════════════════════════════════════
def serial_loop():
    global running
    gui_print(f"[SERIAL] Conectando a {SERIAL_PORT}...")

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2)
        gui_print(f"[SERIAL] Conectado a {SERIAL_PORT}")

        while running:
            while command_queue:
                cmd = command_queue.popleft()
                ser.write((cmd + '\n').encode('utf-8'))
            try:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        if "," in line and (line[0].isdigit() or line[0] == '-'):
                            data_queue.append(line)
                        else:
                            gui_print(f"[ARDUINO] {line}")
            except Exception as e:
                gui_print(f"[SERIAL] Error: {e}")

    except serial.SerialException as e:
        gui_print(f"[SERIAL] No conectado: {e}")
        gui_print("[INFO] Modo teclado — ESPACIO=saque, O=centrar, R=flex")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            gui_print("[SERIAL] Desconectado")


# ══════════════════════════════════════════════════════════════════
#  GUI TKINTER
# ══════════════════════════════════════════════════════════════════
def build_gui(game: FrontonGame):
    global running

    root = tk.Tk()
    root.title("Fronton Haptico — Control")
    root.configure(bg="#12121e")
    root.resizable(False, False)

    tk.Label(root, text="FRONTON HAPTICO",
             bg="#12121e", fg="#e8c84a",
             font=("Consolas", 14, "bold")).pack(pady=(10, 2))

    # ══ SENSIBILIDAD ══════════════════════════════════════════════
    sf = tk.LabelFrame(root,
                       text="  Sensibilidad de inclinacion (roll -> X)  ",
                       bg="#1c1c30", fg="#7ecfff",
                       font=("Consolas", 9, "bold"),
                       bd=2, relief="groove")
    sf.pack(fill=tk.X, padx=12, pady=(8, 0))

    row1 = tk.Frame(sf, bg="#1c1c30")
    row1.pack(fill=tk.X, padx=10, pady=7)

    tk.Label(row1, text="Suave", bg="#1c1c30", fg="#556677",
             font=("Consolas", 8)).pack(side=tk.LEFT)

    sens_val = tk.Label(row1, text=f"{g_sensitivity:.1f}",
                         bg="#1c1c30", fg="#e8c84a",
                         font=("Consolas", 13, "bold"), width=5)

    def on_slider(val):
        global g_sensitivity
        g_sensitivity = float(val)
        sens_val.config(text=f"{g_sensitivity:.1f}")

    tk.Scale(row1, from_=0.3, to=10.0, resolution=0.1,
             orient=tk.HORIZONTAL, showvalue=False,
             command=on_slider, length=210,
             bg="#1c1c30", fg="#e8c84a", troughcolor="#0d0d1a",
             highlightthickness=0, activebackground="#e8c84a"
             ).pack(side=tk.LEFT, padx=6)

    tk.Label(row1, text="Rapido", bg="#1c1c30", fg="#556677",
             font=("Consolas", 8)).pack(side=tk.LEFT)
    sens_val.pack(side=tk.LEFT, padx=5)
    tk.Label(row1, text="m/s/rad", bg="#1c1c30", fg="#7ecfff",
             font=("Consolas", 8)).pack(side=tk.LEFT)

    # ══ BARRA ROLL ═══════════════════════════════════════════════
    tf = tk.LabelFrame(root,
                       text="  Inclinacion actual (Roll IMU)  ",
                       bg="#1c1c30", fg="#7ecfff",
                       font=("Consolas", 9, "bold"),
                       bd=2, relief="groove")
    tf.pack(fill=tk.X, padx=12, pady=(6, 0))

    CW = 360
    canvas = tk.Canvas(tf, width=CW, height=44, bg="#0d0d1a", highlightthickness=0)
    canvas.pack(padx=10, pady=7)

    BW = 300; BX0 = (CW - BW) // 2; BY0, BY1 = 13, 31; CX = BX0 + BW // 2
    canvas.create_rectangle(BX0, BY0, BX0+BW, BY1, fill="#0a0a15", outline="#223344")
    canvas.create_rectangle(CX-25, BY0, CX+25, BY1, fill="#0f2a0f", outline="")
    canvas.create_line(CX, BY0-3, CX, BY1+3, fill="#3a5a3a", dash=(3,3))
    canvas.create_text(BX0-10, (BY0+BY1)//2, text="<", fill="#667788",
                       font=("Consolas", 9))
    canvas.create_text(BX0+BW+10, (BY0+BY1)//2, text=">", fill="#667788",
                       font=("Consolas", 9))

    ind = canvas.create_rectangle(CX-6, BY0+1, CX+6, BY1-1, fill="#44ee88", outline="")
    roll_var = tk.StringVar(value="0.0 deg  |  X: 0.00 m")
    tk.Label(tf, textvariable=roll_var, bg="#1c1c30", fg="#aaccff",
             font=("Consolas", 9)).pack(pady=(0, 4))

    def update_bar():
        roll = game.current_roll
        norm = max(-1.0, min(1.0, roll / (math.pi / 2.0)))
        x = CX + int(norm * BW / 2)
        canvas.coords(ind, x-6, BY0+1, x+6, BY1-1)
        color = "#ff3333" if abs(norm) > 0.75 else ("#ffaa22" if abs(norm) > 0.45 else "#44ee88")
        canvas.itemconfig(ind, fill=color)
        roll_var.set(f"Roll: {math.degrees(roll):+.1f} deg  |  "
                     f"X: {game.glove_x:+.2f} m  |  "
                     f"Puntos: {game.score}")
        if running:
            root.after(35, update_bar)

    update_bar()

    # ══ BOTONES ══════════════════════════════════════════════════
    bf = tk.Frame(root, bg="#12121e")
    bf.pack(pady=6)

    for label, cmd, color in [
        ("Reset IMU",    lambda: (game.imu.reset_orientation(),
                                  setattr(game, 'glove_x', 0.0)),          "#7ecfff"),
        ("Nuevo saque",  lambda: (setattr(game, 'serve_ready', True),
                                  setattr(game, 'ball_active', False)),    "#a8f0a0"),
        ("Reset score",  lambda: setattr(game, 'score', 0),                "#f0e080"),
        ("Recal. flex",  lambda: game.reset_calibration(),                  "#ffaa66"),
    ]:
        tk.Button(bf, text=label, command=cmd,
                  bg="#1c2a3a", fg=color,
                  font=("Consolas", 9, "bold"),
                  relief="flat", padx=7, pady=4
                  ).pack(side=tk.LEFT, padx=3)

    # ══ LOG ══════════════════════════════════════════════════════
    lf = tk.LabelFrame(root, text="  Log  ",
                        bg="#12121e", fg="#334455",
                        font=("Consolas", 8), bd=1, relief="flat")
    lf.pack(fill=tk.BOTH, expand=True, padx=12, pady=(4, 0))

    log = tk.Text(lf, height=11, width=62,
                   bg="#0d0d1a", fg="#99b8cc",
                   font=("Consolas", 9),
                   insertbackground="white", relief="flat", bd=4)
    sb = tk.Scrollbar(lf, command=log.yview)
    log.config(yscrollcommand=sb.set)
    sb.pack(side=tk.RIGHT, fill=tk.Y)
    log.pack(fill=tk.BOTH, expand=True)

    ce = tk.Frame(root, bg="#12121e")
    ce.pack(fill=tk.X, padx=12, pady=8)
    tk.Label(ce, text=">", bg="#12121e", fg="#e8c84a",
             font=("Consolas", 11, "bold")).pack(side=tk.LEFT)
    ent = tk.Entry(ce, width=46, bg="#1c1c30", fg="white",
                    font=("Consolas", 10), insertbackground="white",
                    relief="flat", bd=4)
    ent.pack(side=tk.LEFT, padx=4)

    def send_cmd(event=None):
        global running
        cmd = ent.get().strip()
        if not cmd: return
        ent.delete(0, tk.END)
        gui_print(f"> {cmd}")
        if cmd.lower() == "quit":
            running = False; root.quit(); return
        if cmd.lower().startswith("config "):
            parts = cmd.split()
            if len(parts) >= 4:
                zone = parts[1].upper()
                try:
                    m_pos = int(parts[2])
                    stype = parts[3].upper()
                    game.haptic_config.configure_zone(zone, m_pos, stype)
                except ValueError:
                    gui_print("[CONFIG] Formato invalido")
            return
        command_queue.append(cmd.upper())

    ent.bind("<Return>", send_cmd)
    tk.Button(ce, text="Enviar", command=send_cmd,
              bg="#e8c84a", fg="#12121e",
              font=("Consolas", 9, "bold"),
              relief="flat", padx=6).pack(side=tk.LEFT)

    def poll_log():
        try:
            while not gui_queue.empty():
                _, msg = gui_queue.get_nowait()
                log.insert(tk.END, msg + "\n")
                log.see(tk.END)
        except Exception:
            pass
        if running:
            root.after(80, poll_log)

    poll_log()

    def on_close():
        global running
        running = False
        root.quit()

    root.protocol("WM_DELETE_WINDOW", on_close)
    tk.Button(root, text="Salir", command=on_close,
              bg="#cc3344", fg="white",
              font=("Consolas", 10, "bold"),
              relief="flat", padx=10, pady=4
              ).pack(pady=(0, 10))

    return root


# ══════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    game = FrontonGame(urdf_filename="schunk_svh_hand_right.urdf")

    t_serial = threading.Thread(target=serial_loop, daemon=True)
    t_serial.start()

    t_game = threading.Thread(target=game.run, daemon=True)
    t_game.start()

    root = build_gui(game)
    root.mainloop()

    running = False
    time.sleep(0.4)
    try:
        if p.isConnected():
            p.disconnect()
    except Exception:
        pass