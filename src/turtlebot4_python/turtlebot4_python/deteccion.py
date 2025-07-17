import cv2
import mediapipe as mp
import numpy as np
import time
from PyQt5.QtCore import QTimer, QObject, pyqtSignal

class QtHandGestureDetector(QObject):
    """Detector de gestos compatible con PyQt5 - No bloquea la GUI"""
    
    # Señales para comunicación con GUI
    gesture_detected = pyqtSignal(int, str)  # finger_count, command
    dance_mode_changed = pyqtSignal(bool)    # activate/deactivate
    status_updated = pyqtSignal(str, bool)   # message, is_error
    
    def __init__(self):
        super().__init__()
        self.setup_mediapipe()
        self.setup_commands()
        self.setup_camera()
        
        # Timer para procesamiento no bloqueante
        self.timer = QTimer()
        self.timer.timeout.connect(self.process_frame)
        
        # Estado
        self.is_running = False
        self.cap = None
        
        self.status_updated.emit("Detector de gestos inicializado", False)
        
    def setup_mediapipe(self):
        """Configurar MediaPipe"""
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        
        # Configuración optimizada para PyQt5
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5,
            model_complexity=0
        )
        
    def setup_commands(self):
        """Configurar comandos"""
        self.last_command_time = 0
        self.command_cooldown = 2.0
        self.last_finger_count = 0
        self.finger_count_history = []
        self.stability_threshold = 5
        
        self.finger_to_command = {
            1: {"name": "Punto A", "command": "1"},
            2: {"name": "Punto B", "command": "2"},
            3: {"name": "Punto C", "command": "3"},
            4: {"name": "Punto D", "command": "4"},
            5: {"name": "Modo Baile", "command": "dance"}
        }
        
        self.music_mode_active = False
        self.music_mode_start_time = 0
        self.music_mode_duration = 10.0
        
    def setup_camera(self):
        """Configurar cámara"""
        self.cap = None
        self.window_name = 'Robot Gesture Controller'
        
    def start_detection(self):
        """Iniciar detección sin bloquear GUI"""
        if self.is_running:
            return False
            
        self.status_updated.emit("Iniciando cámara...", False)
        
        # Configurar cámara
        backends_to_try = [cv2.CAP_V4L2, cv2.CAP_DSHOW, cv2.CAP_ANY]
        
        for backend in backends_to_try:
            try:
                self.cap = cv2.VideoCapture(0, backend)
                if self.cap and self.cap.isOpened():
                    self.status_updated.emit(f"✅ Cámara abierta", False)
                    break
                if self.cap:
                    self.cap.release()
            except:
                continue
        
        if not self.cap or not self.cap.isOpened():
            self.status_updated.emit("ERROR: No se pudo abrir la cámara", True)
            return False
        
        # Configurar cámara
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        # Leer frames iniciales
        for _ in range(5):
            self.cap.read()
        
        # Crear ventana
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1280, 720)
        
        # Iniciar timer (30 FPS)
        self.is_running = True
        self.timer.start(33)  # ~30 FPS
        
        self.status_updated.emit("✅ Detección iniciada - Muestra tu mano", False)
        return True
    
    def stop_detection(self):
        """Detener detección"""
        self.is_running = False
        self.timer.stop()
        
        if self.cap:
            self.cap.release()
            self.cap = None
            
        cv2.destroyAllWindows()
        
        if self.music_mode_active:
            self.music_mode_active = False
            self.dance_mode_changed.emit(False)
        
        self.status_updated.emit("Detección detenida", False)
    
    def process_frame(self):
        """Procesar un frame (llamado por QTimer)"""
        if not self.is_running or not self.cap:
            return
            
        # Leer frame
        ret, frame = self.cap.read()
        if not ret:
            return
        
        try:
            # Procesar frame
            frame = cv2.flip(frame, 1)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_rgb.flags.writeable = False
            results = self.hands.process(frame_rgb)
            frame_rgb.flags.writeable = True
            
            finger_count = 0
            finger_details = []
            
            # Procesar resultados
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # Dibujar landmarks
                    self.mp_drawing.draw_landmarks(
                        frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS,
                        self.mp_drawing.DrawingSpec(color=(0,0,255), thickness=2, circle_radius=2),
                        self.mp_drawing.DrawingSpec(color=(0,255,0), thickness=2)
                    )
                    
                    # Contar dedos
                    finger_count, finger_details = self.count_extended_fingers(hand_landmarks.landmark)
                    
                    # Dibujar indicadores
                    self.draw_finger_indicators(frame, hand_landmarks.landmark, finger_count, finger_details)
                    
                    # Manejar comandos
                    self.handle_robot_command(finger_count)
            
            # Dibujar interfaz
            self.draw_ui_overlay(frame, finger_count, finger_details)
            
            # Mostrar frame
            cv2.imshow(self.window_name, frame)
            
            # Verificar teclas (no bloqueante)
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):  # ESC o Q
                self.stop_detection()
                
        except Exception as e:
            self.status_updated.emit(f"Error procesando frame: {e}", True)
    
    def is_finger_extended(self, landmarks, finger_tip_idx, finger_pip_idx, finger_mcp_idx=None):
        """Determinar si un dedo está extendido"""
        tip_y = landmarks[finger_tip_idx].y
        pip_y = landmarks[finger_pip_idx].y
        
        if finger_tip_idx == self.mp_hands.HandLandmark.THUMB_TIP:
            tip_x = landmarks[finger_tip_idx].x
            mcp_x = landmarks[self.mp_hands.HandLandmark.THUMB_MCP].x
            return abs(tip_x - mcp_x) > 0.04
        
        if finger_mcp_idx:
            mcp_y = landmarks[finger_mcp_idx].y
            return tip_y < pip_y and tip_y < mcp_y
        else:
            return tip_y < pip_y
    
    def count_extended_fingers(self, landmarks):
        """Contar dedos extendidos"""
        extended_fingers = 0
        finger_details = []
        
        fingers_to_check = [
            (self.mp_hands.HandLandmark.THUMB_TIP, self.mp_hands.HandLandmark.THUMB_IP, None, "Pulgar"),
            (self.mp_hands.HandLandmark.INDEX_FINGER_TIP, self.mp_hands.HandLandmark.INDEX_FINGER_PIP, 
             self.mp_hands.HandLandmark.INDEX_FINGER_MCP, "Índice"),
            (self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP, self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP,
             self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP, "Medio"),
            (self.mp_hands.HandLandmark.RING_FINGER_TIP, self.mp_hands.HandLandmark.RING_FINGER_PIP,
             self.mp_hands.HandLandmark.RING_FINGER_MCP, "Anular"),
            (self.mp_hands.HandLandmark.PINKY_TIP, self.mp_hands.HandLandmark.PINKY_PIP,
             self.mp_hands.HandLandmark.PINKY_MCP, "Meñique")
        ]
        
        for tip, pip, mcp, name in fingers_to_check:
            if self.is_finger_extended(landmarks, tip, pip, mcp):
                extended_fingers += 1
                finger_details.append(name)
        
        return extended_fingers, finger_details
    
    def is_gesture_stable(self, finger_count):
        """Verificar estabilidad del gesto"""
        self.finger_count_history.append(finger_count)
        
        if len(self.finger_count_history) > self.stability_threshold:
            self.finger_count_history.pop(0)
        
        if len(self.finger_count_history) >= self.stability_threshold:
            return all(count == finger_count for count in self.finger_count_history)
        
        return False
    
    def handle_robot_command(self, finger_count):
        """Manejar comandos del robot"""
        current_time = time.time()
        
        # Verificar modo baile
        if self.music_mode_active:
            if current_time - self.music_mode_start_time > self.music_mode_duration:
                self.music_mode_active = False
                self.dance_mode_changed.emit(False)
                self.status_updated.emit("🕺 MODO BAILE DESACTIVADO", False)
        
        # Verificar si enviar comando
        if (current_time - self.last_command_time > self.command_cooldown and 
            finger_count in self.finger_to_command and
            finger_count != self.last_finger_count and
            self.is_gesture_stable(finger_count)):
            
            command_info = self.finger_to_command[finger_count]
            
            if finger_count == 5:
                # Modo baile
                if not self.music_mode_active:
                    self.music_mode_active = True
                    self.music_mode_start_time = current_time
                    self.dance_mode_changed.emit(True)
                    self.status_updated.emit("🕺 MODO BAILE ACTIVADO!", False)
                else:
                    self.music_mode_active = False
                    self.dance_mode_changed.emit(False)
                    self.status_updated.emit("🕺 MODO BAILE DESACTIVADO", False)
            else:
                # Comando normal
                self.gesture_detected.emit(finger_count, command_info['command'])
                self.status_updated.emit(f"🤖 {finger_count} dedos → {command_info['name']}", False)
            
            self.last_command_time = current_time
            self.last_finger_count = finger_count
    
    def draw_finger_indicators(self, frame, landmarks, finger_count, finger_details):
        """Dibujar indicadores de dedos"""
        h, w, c = frame.shape
        
        fingers_info = [
            (self.mp_hands.HandLandmark.THUMB_TIP, "Pulgar", (0, 255, 0)),
            (self.mp_hands.HandLandmark.INDEX_FINGER_TIP, "Índice", (0, 0, 255)),
            (self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP, "Medio", (255, 0, 0)),
            (self.mp_hands.HandLandmark.RING_FINGER_TIP, "Anular", (255, 255, 0)),
            (self.mp_hands.HandLandmark.PINKY_TIP, "Meñique", (0, 255, 255))
        ]
        
        for finger_tip, finger_name, color in fingers_info:
            pos = landmarks[finger_tip]
            x, y = int(pos.x * w), int(pos.y * h)
            
            if finger_name in finger_details:
                cv2.circle(frame, (x, y), 15, color, -1)
                cv2.putText(frame, finger_name[:3], (x-20, y-25), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            else:
                cv2.circle(frame, (x, y), 8, color, 2)
    
    def draw_ui_overlay(self, frame, finger_count, finger_details):
        """Dibujar interfaz"""
        h, w = frame.shape[:2]
        
        # Fondo
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (w-10, 220), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        # Información
        cv2.putText(frame, f"Dedos detectados: {finger_count}", (20, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
        
        if finger_count in self.finger_to_command:
            command_info = self.finger_to_command[finger_count]
            color = (0, 255, 255) if finger_count != 5 else (255, 0, 255)
            cv2.putText(frame, f"Comando: {command_info['name']}", (20, 90), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
        
        if finger_details:
            details_text = f"Dedos: {', '.join(finger_details)}"
            cv2.putText(frame, details_text, (20, 130), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
        
        # Cooldown
        current_time = time.time()
        time_remaining = max(0, self.command_cooldown - (current_time - self.last_command_time))
        if time_remaining > 0:
            cv2.putText(frame, f"Siguiente comando en: {time_remaining:.1f}s", (20, 170), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 100, 100), 2)
        
        # Modo baile
        if self.music_mode_active:
            remaining_dance = self.music_mode_duration - (current_time - self.music_mode_start_time)
            cv2.putText(frame, f"🕺 BAILE ACTIVO ({remaining_dance:.1f}s) 🕺", (w//2 - 200, h - 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 255), 3)
        
        # Instrucciones
        instructions = [
            "1 dedo = Punto A    2 dedos = Punto B    3 dedos = Punto C",
            "4 dedos = Punto D    5 dedos = Modo Baile    ESC/Q = Salir"
        ]
        
        for i, instruction in enumerate(instructions):
            cv2.putText(frame, instruction, (20, h - 80 + i*30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)