import numpy as np
import pybullet as p
import pybullet_data
import time
import cv2
import mediapipe as mp
import threading
import math

class MediaPipePosTracker:
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            model_complexity=1,              # Mejor precisión
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7,
            max_num_hands=1
        )
        self.cap = cv2.VideoCapture(0)
        self.position = [0.0, 0.0, 0.4]
        self.hand_detected = False
        self.running = True

        # Parámetros de escala y offset (ajústalos según tu setup)
        self.scale_x = 1.8
        self.scale_z = 1.2
        self.offset_x = -0.9
        self.offset_z = 0.3

        # Para estimar profundidad por tamaño de mano
        self.base_hand_size = None      # Se calibra la primera vez que detecta mano
        self.depth_scale = 2.0          # Sensibilidad del movimiento adelante/atrás
        self.base_depth = 0.0           # Profundidad base (Y en PyBullet)

        # Suavizado
        self.smoothed_pos = np.array([0.0, 0.0, 0.4])

    def start(self):
        if not self.cap.isOpened():
            print("[ERROR] No se pudo abrir la cámara")
            return
        print("[VISION] Cámara iniciada - Mueve tu mano para controlar la esfera en 3D")
        threading.Thread(target=self._loop, daemon=True).start()

    def _calculate_hand_size(self, landmarks):
        """Calcula una medida aproximada del tamaño de la mano (distancia muñeca a dedo medio)"""
        wrist = landmarks[0]
        middle_tip = landmarks[12]  # Punta del dedo medio
        dx = wrist.x - middle_tip.x
        dy = wrist.y - middle_tip.y
        dz = wrist.z - middle_tip.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def _loop(self):
        while self.running and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                continue

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(frame_rgb)

            if results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]
                wrist = hand_landmarks.landmark[0]

                # Calibrar tamaño base la primera vez
                if self.base_hand_size is None:
                    self.base_hand_size = self._calculate_hand_size(hand_landmarks.landmark)
                    print(f"[VISION] Mano calibrada - tamaño base: {self.base_hand_size:.3f}")

                current_size = self._calculate_hand_size(hand_landmarks.landmark)

                # Estimar profundidad (Y) a partir del tamaño relativo de la mano
                if self.base_hand_size > 0:
                    size_ratio = self.base_hand_size / current_size
                    depth_y = self.base_depth + (size_ratio - 1.0) * self.depth_scale
                else:
                    depth_y = self.base_depth

                # Posición X y Z (las de siempre, pero mejoradas)
                pb_x = (wrist.x - 0.5) * -self.scale_x + self.offset_x
                pb_z = (1.0 - wrist.y) * self.scale_z + self.offset_z
                pb_y = depth_y

                new_pos = np.array([pb_x, pb_y, pb_z])

                # Suavizado fuerte para evitar temblores
                self.smoothed_pos = self.smoothed_pos * 0.8 + new_pos * 0.2

                self.position = self.smoothed_pos.tolist()
                self.hand_detected = True
            else:
                self.hand_detected = False

            time.sleep(0.01)

        self.cap.release()

    def get_position(self):
        return self.position, self.hand_detected


# === PYBULLET SETUP ===
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")

# Esfera que seguirá tu mano
sphere_id = p.loadURDF("sphere_small.urdf", [0, 0, 0.4])
p.changeVisualShape(sphere_id, -1, rgbaColor=[0.1, 0.6, 1.0, 1])  # Azul bonito

# Cámara cómoda para ver el movimiento 3D
p.resetDebugVisualizerCamera(cameraDistance=1.2, cameraYaw=50, cameraPitch=-25, cameraTargetPosition=[0, 0, 0.3])

tracker = MediaPipePosTracker()
tracker.start()

prev_pos = None
print("\n=== CONTROLES ===")
print("• Mueve la mano de lado a lado → movimiento X")
print("• Sube/baja la mano → movimiento Z (altura)")
print("• Acerca o aleja la mano de la cámara → movimiento Y (profundidad)")
print("• La trayectoria se dibuja en rojo")
print("Presiona 'q' para salir\n")

while p.isConnected():
    pos, detected = tracker.get_position()

    if detected:
        p.resetBasePositionAndOrientation(sphere_id, pos, [0, 0, 0, 1])

        # Dibujar trayectoria permanente
        if prev_pos is not None:
            p.addUserDebugLine(prev_pos, pos, [1, 0.2, 0.2], 3, lifeTime=0)

        prev_pos = pos[:]

    keys = p.getKeyboardEvents()
    if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
        break

    p.stepSimulation()
    time.sleep(1/240)

tracker.running = False
p.disconnect()
print("Adiós!")