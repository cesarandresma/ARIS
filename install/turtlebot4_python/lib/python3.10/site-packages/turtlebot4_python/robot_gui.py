#!usr/bin/env python

from PyQt5 import QtWidgets, QtCore, QtWebEngineWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QStackedWidget, QWidget, QVBoxLayout
from PyQt5.QtCore import pyqtSlot, Qt, QTimer, QUrl
from PyQt5.QtWebEngineWidgets import QWebEngineView
import sys
import time
import tempfile
import os

# ============================================
# IMPORTS ROS - 
# ============================================
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
#from turtlebot4_python import audio
#from turtlebot4_python import deteccion

# Importar módulos del proyecto 
try:
     from turtlebot4_python.audio import VoiceAssistantBackend
     AUDIO_AVAILABLE = True
except ImportError:
     print("ADVERTENCIA: No se pudo importar VoiceAssistantBackend")
     AUDIO_AVAILABLE = False

try:
     from turtlebot4_python.deteccion import QtHandGestureDetector
     GESTURES_AVAILABLE = True
except ImportError:
     print("ADVERTENCIA: No se pudo importar QtHandGestureDetector")
     GESTURES_AVAILABLE = False


class AnimatedFaceWidget(QWidget):
    """Widget que muestra la cara animada con botón simple para continuar"""
    
    face_clicked = QtCore.pyqtSignal()
    
    def __init__(self):
        super().__init__()
        print("🟢 Iniciando AnimatedFaceWidget...")
        self.initUI()
        
    def initUI(self):
        # Layout principal
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Crear WebView para mostrar HTML
        self.web_view = QWebEngineView()
        
        # Buscar archivo rostro.html
        #html_file_path = self.find_html_file()
        html_file_path = "/home/ister/turtlebot4_ws/src/turtlebot4_python/turtlebot4_python/rostro.html"
        
        if html_file_path:
            # Cargar archivo HTML local
            file_url = QUrl.fromLocalFile(html_file_path)
            self.web_view.load(file_url)
            print(f"✅ Cargando rostro.html desde: {html_file_path}")
        else:
            # Fallback: HTML básico si no encuentra el archivo
            self.load_fallback_html()
            print("⚠️ rostro.html no encontrado, usando HTML básico")
        
        layout.addWidget(self.web_view)
        self.setLayout(layout)
        
        # Crear botón flotante después de que se cargue la página
        QTimer.singleShot(1000, self.create_exit_button)
        
    def create_exit_button(self):
        """Crear el botón flotante para salir"""
        print("🟢 Creando botón de salida...")
        
        self.exit_button = QtWidgets.QPushButton("▶", self)
        
        # Estilo del botón
        self.exit_button.setStyleSheet("""
            QPushButton {
                background-color: rgba(231, 76, 60, 0.95);
                color: white;
                border: 3px solid rgba(255, 255, 255, 0.9);
                border-radius: 30px;
                font-size: 22px;
                font-weight: bold;
                padding: 0px;
            }
            QPushButton:hover {
                background-color: rgba(231, 76, 60, 1.0);
                border: 3px solid white;
                transform: scale(1.05);
            }
            QPushButton:pressed {
                background-color: rgba(192, 57, 43, 1.0);
            }
        """)
        
        # Configurar botón
        self.exit_button.setFixedSize(65, 65)
        self.exit_button.setToolTip("Continuar a la interfaz principal")
        self.exit_button.clicked.connect(self.on_exit_button_clicked)
        
        # Posicionar y mostrar
        self.position_exit_button()
        self.exit_button.show()
        self.exit_button.raise_()  # Traer al frente
        
        print("✅ Botón de salida creado")
        
    def position_exit_button(self):
        """Posicionar el botón en la esquina inferior derecha"""
        if hasattr(self, 'exit_button'):
            margin = 25
            x = self.width() - 65 - margin
            y = self.height() - 65 - margin
            self.exit_button.move(x, y)
    
    def resizeEvent(self, event):
        """Reposicionar el botón cuando cambie el tamaño de la ventana"""
        super().resizeEvent(event)
        self.position_exit_button()
    
    def on_exit_button_clicked(self):
        """Cuando se hace clic en el botón de salida"""
        print("🎯 ¡Botón de salida clickeado!")
        self.face_clicked.emit()
    
    def find_html_file(self):
        """Buscar archivo rostro.html en varios lugares"""
        possible_paths = [
            "rostro.html",
            "./rostro.html",
            os.path.join(os.getcwd(), "rostro.html"),
            os.path.join(os.path.dirname(__file__), "rostro.html"),
        ]
        
        for path in possible_paths:
            abs_path = os.path.abspath(path)
            if os.path.exists(abs_path):
                return abs_path
        
        return None
    
    def load_fallback_html(self):
        """Cargar HTML básico si no encuentra rostro.html"""
        fallback_html = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>ARIS - Cara Animada</title>
    <style>
        body {
            display: flex;
            flex-direction: column;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            margin: 0;
            font-family: Arial, sans-serif;
            color: white;
            text-align: center;
            overflow: hidden;
        }
        
        .title {
            font-size: 4rem;
            font-weight: bold;
            margin-bottom: 2rem;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
            animation: pulse 2s infinite;
        }
        
        .subtitle {
            font-size: 1.8rem;
            margin-bottom: 3rem;
            text-shadow: 1px 1px 2px rgba(0,0,0,0.3);
            opacity: 0.9;
        }
        
        .face-container {
            width: 280px;
            height: 200px;
            background-color: #2c3e50;
            border-radius: 25px;
            position: relative;
            margin-bottom: 3rem;
            box-shadow: 0 10px 30px rgba(0,0,0,0.3);
        }
        
        .eye {
            width: 50px;
            height: 50px;
            background-color: white;
            border-radius: 50%;
            position: absolute;
            top: 60px;
        }
        
        .left-eye { left: 60px; }
        .right-eye { right: 60px; }
        
        .pupil {
            width: 20px;
            height: 20px;
            background-color: #2c3e50;
            border-radius: 50%;
            position: absolute;
            top: 15px;
            left: 15px;
            animation: lookAround 4s infinite ease-in-out;
        }
        
        .mouth {
            width: 40px;
            height: 12px;
            border: 3px solid white;
            border-color: white transparent transparent transparent;
            border-radius: 0 0 40px 40px;
            position: absolute;
            bottom: 50px;
            left: 50%;
            margin-left: -20px;
        }
        
        .instruction {
            font-size: 1.3rem;
            margin-top: 2rem;
            animation: bounce 2s infinite;
        }
        
        @keyframes pulse {
            0%, 100% { transform: scale(1); }
            50% { transform: scale(1.05); }
        }
        
        @keyframes bounce {
            0%, 20%, 50%, 80%, 100% { transform: translateY(0); }
            40% { transform: translateY(-10px); }
            60% { transform: translateY(-5px); }
        }
        
        @keyframes lookAround {
            0%, 100% { transform: translate(0px, 0px); }
            25% { transform: translate(-8px, 2px); }
            50% { transform: translate(8px, -2px); }
            75% { transform: translate(-4px, -4px); }
        }
    </style>
</head>
<body>
    <div class="title">ARIS</div>
    <div class="subtitle">Asistente Robótico Inteligente</div>
    
    <div class="face-container">
        <div class="eye left-eye">
            <div class="pupil"></div>
        </div>
        <div class="eye right-eye">
            <div class="pupil"></div>
        </div>
        <div class="mouth"></div>
    </div>
    
    <div class="instruction">HAZ CLIC EN EL BOTÓN ▶ PARA CONTINUAR</div>
</body>
</html>
        """
        self.web_view.setHtml(fallback_html)
        
    def mousePressEvent(self, event):
        """Fallback: clic en cualquier lugar del widget también funciona"""
        print("🖱️ Clic detectado en el widget completo")
        self.face_clicked.emit()


class MainInterfaceWidget(QWidget):
    """Widget de la interfaz principal con botones"""
    
    def __init__(self, parent_window):
        super().__init__()
        self.parent_window = parent_window
        self.initUI()
        
        # Variables para control por gestos
        self.gesture_detector = None
        self.gesture_active = False
        
    def initUI(self):
        # Configurar el fondo con gradiente
        self.setStyleSheet("""
            QWidget {
                background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,
                                          stop: 0 #667eea, stop: 1 #764ba2);
            }
        """)
        
        # Label de estado (perfectamente centrado)
        self.label = QtWidgets.QLabel(self)
        self.label.setText("🤖 Selecciona una opción")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("""
            QLabel {
                color: white;
                background-color: rgba(0, 0, 0, 120);
                border-radius: 15px;
                padding: 15px 20px;
                font-size: 18px;
                font-weight: bold;
                border: 2px solid rgba(255, 255, 255, 0.2);
            }
        """)

        # Estilo base para botones A, B, C, D
        button_style = """
            QPushButton {
                background-color: rgba(52, 152, 219, 0.85);
                color: white;
                border: 3px solid rgba(255, 255, 255, 0.8);
                border-radius: 50px;
                font-size: 32px;
                font-weight: bold;
                text-shadow: 2px 2px 4px rgba(0, 0, 0, 0.5);
            }
            QPushButton:hover {
                background-color: rgba(52, 152, 219, 0.95);
                border: 3px solid white;
                transform: scale(1.08);
                box-shadow: 0 8px 20px rgba(0, 0, 0, 0.3);
            }
            QPushButton:pressed {
                background-color: rgba(41, 128, 185, 1.0);
                transform: scale(1.02);
            }
        """

        # ============================================
        # FILA SUPERIOR: BOTONES A, B, C, D (CENTRADOS)
        # ============================================
        
        self.button_width = 130
        self.button_height = 130
        self.spacing = 40
        self.top_y = 150
        
        # Crear botones A, B, C, D
        self.b1 = QtWidgets.QPushButton(self)
        self.b1.setText("A")
        self.b1.clicked.connect(lambda: self.parent_window.button_clicked("A", "1"))
        self.b1.setFixedSize(self.button_width, self.button_height)
        self.b1.setStyleSheet(button_style)

        self.b2 = QtWidgets.QPushButton(self)
        self.b2.setText("B")
        self.b2.clicked.connect(lambda: self.parent_window.button_clicked("B", "2"))
        self.b2.setFixedSize(self.button_width, self.button_height)
        self.b2.setStyleSheet(button_style)

        self.b3 = QtWidgets.QPushButton(self)
        self.b3.setText("C")
        self.b3.clicked.connect(lambda: self.parent_window.button_clicked("C", "3"))
        self.b3.setFixedSize(self.button_width, self.button_height)
        self.b3.setStyleSheet(button_style)

        self.b4 = QtWidgets.QPushButton(self)
        self.b4.setText("D")
        self.b4.clicked.connect(lambda: self.parent_window.button_clicked("D", "4"))
        self.b4.setFixedSize(self.button_width, self.button_height)
        self.b4.setStyleSheet(button_style)

        # ============================================
        # FILA INFERIOR: HABLAR, HOME, GESTOS (CENTRADOS)
        # ============================================
        
        self.bottom_y = 400
        
        # Botón HABLAR
        self.voice_width = 170
        self.voice_height = 150
        
        self.voice_button = QtWidgets.QPushButton(self)
        self.voice_button.setText("🎤\nHABLAR")
        self.voice_button.clicked.connect(self.parent_window.button_clicked_voice)
        self.voice_button.setFixedSize(self.voice_width, self.voice_height)
        
        voice_style = """
            QPushButton {
                background-color: rgba(39, 174, 96, 0.85);
                color: white;
                border: 3px solid rgba(255, 255, 255, 0.8);
                border-radius: 75px;
                font-size: 22px;
                font-weight: bold;
                padding: 10px;
                text-shadow: 2px 2px 4px rgba(0, 0, 0, 0.5);
            }
            QPushButton:hover {
                background-color: rgba(39, 174, 96, 0.95);
                border: 3px solid white;
                transform: scale(1.08);
                box-shadow: 0 10px 25px rgba(0, 0, 0, 0.3);
            }
            QPushButton:pressed {
                background-color: rgba(34, 153, 84, 1.0);
                transform: scale(1.02);
            }
            QPushButton:disabled {
                background-color: rgba(128, 128, 128, 0.6);
                color: rgba(255, 255, 255, 0.6);
                border: 3px solid rgba(255, 255, 255, 0.3);
                text-shadow: 1px 1px 2px rgba(0, 0, 0, 0.5);
            }
        """
        self.voice_button.setStyleSheet(voice_style)

        # Botón HOME
        self.b5 = QtWidgets.QPushButton(self)
        self.b5.setText("HOME")
        self.b5.clicked.connect(lambda: self.parent_window.button_clicked("HOME", "5"))
        self.b5.setFixedSize(self.button_width, self.button_height)
        
        home_style = """
            QPushButton {
                background-color: rgba(231, 76, 60, 0.85);
                color: white;
                border: 3px solid rgba(255, 255, 255, 0.8);
                border-radius: 50px;
                font-size: 20px;
                font-weight: bold;
                text-shadow: 2px 2px 4px rgba(0, 0, 0, 0.5);
            }
            QPushButton:hover {
                background-color: rgba(231, 76, 60, 0.95);
                border: 3px solid white;
                transform: scale(1.08);
                box-shadow: 0 8px 20px rgba(0, 0, 0, 0.3);
            }
            QPushButton:pressed {
                background-color: rgba(192, 57, 43, 1.0);
                transform: scale(1.02);
            }
        """
        self.b5.setStyleSheet(home_style)

        # Botón GESTOS
        self.gesture_button = QtWidgets.QPushButton(self)
        self.gesture_button.setText("👋\nGESTOS")
        self.gesture_button.clicked.connect(self.button_clicked_gestures)
        self.gesture_button.setFixedSize(self.button_width, self.button_height)
        
        gesture_style = """
            QPushButton {
                background-color: rgba(142, 68, 173, 0.85);
                color: white;
                border: 3px solid rgba(255, 255, 255, 0.8);
                border-radius: 50px;
                font-size: 16px;
                font-weight: bold;
                text-shadow: 2px 2px 4px rgba(0, 0, 0, 0.5);
            }
            QPushButton:hover {
                background-color: rgba(142, 68, 173, 0.95);
                border: 3px solid white;
                transform: scale(1.08);
                box-shadow: 0 8px 20px rgba(0, 0, 0, 0.3);
            }
            QPushButton:pressed {
                background-color: rgba(125, 60, 152, 1.0);
                transform: scale(1.02);
            }
        """
        #GESTURES_AVAILABLE = True
        if GESTURES_AVAILABLE:
            self.gesture_button.setStyleSheet(gesture_style)
        else:
            self.gesture_button.setEnabled(False)
            self.gesture_button.setStyleSheet("""
                QPushButton {
                    background-color: rgba(128, 128, 128, 0.6);
                    color: rgba(255, 255, 255, 0.6);
                    border: 3px solid rgba(255, 255, 255, 0.3);
                    border-radius: 50px;
                    font-size: 16px;
                    text-shadow: 1px 1px 2px rgba(0, 0, 0, 0.5);
                }
            """)

        # Botón CARA (mantiene posición fija)
        self.face_button = QtWidgets.QPushButton(self)
        self.face_button.setText("👁\nCara")
        self.face_button.clicked.connect(self.parent_window.show_animated_face)
        self.face_button.setGeometry(20, 20, 100, 50)
        self.face_button.setStyleSheet("""
            QPushButton {
                background-color: rgba(52, 73, 94, 0.85);
                color: white;
                border: 3px solid rgba(255, 255, 255, 0.8);
                border-radius: 25px;
                font-size: 14px;
                font-weight: bold;
                text-shadow: 1px 1px 2px rgba(0, 0, 0, 0.5);
            }
            QPushButton:hover {
                background-color: rgba(52, 73, 94, 0.95);
                border: 3px solid white;
                transform: scale(1.08);
                box-shadow: 0 6px 15px rgba(0, 0, 0, 0.3);
            }
            QPushButton:pressed {
                background-color: rgba(44, 62, 80, 1.0);
                transform: scale(1.02);
            }
        """)

        # Deshabilitar botón de voz si no está disponible 
        #AUDIO_AVAILABLE = True
        if not AUDIO_AVAILABLE:
            self.voice_button.setEnabled(False)
            self.voice_button.setText("🎤\nNo disponible")

        # Posicionar elementos inicialmente
        self.position_all_elements()

    def position_all_elements(self):
        """Posicionar todos los elementos centrados automáticamente"""
        if self.width() <= 0:
            QTimer.singleShot(100, self.position_all_elements)
            return
            
        center_x = self.width() // 2
        
        # ============================================
        # POSICIONAR FILA SUPERIOR (A, B, C, D)
        # ============================================
        
        # Total width de 4 botones: 4*130 + 3*40 = 640px
        total_width_top = 4 * self.button_width + 3 * self.spacing
        start_x_top = center_x - total_width_top // 2
        
        self.b1.move(start_x_top, self.top_y)
        self.b2.move(start_x_top + (self.button_width + self.spacing), self.top_y)
        self.b3.move(start_x_top + 2 * (self.button_width + self.spacing), self.top_y)
        self.b4.move(start_x_top + 3 * (self.button_width + self.spacing), self.top_y)
        
        # ============================================
        # POSICIONAR FILA INFERIOR (HABLAR, HOME, GESTOS)
        # ============================================
        
        # Espaciado entre botones inferiores
        bottom_spacing = 60
        
        # Total width: 170 + 60 + 130 + 60 + 130 = 550px
        total_width_bottom = self.voice_width + bottom_spacing + self.button_width + bottom_spacing + self.button_width
        start_x_bottom = center_x - total_width_bottom // 2
        
        # HABLAR (izquierda)
        self.voice_button.move(start_x_bottom, self.bottom_y)
        
        # HOME (centro)
        home_x = start_x_bottom + self.voice_width + bottom_spacing
        self.b5.move(home_x, self.bottom_y)
        
        # GESTOS (derecha)
        gestos_x = home_x + self.button_width + bottom_spacing
        self.gesture_button.move(gestos_x, self.bottom_y)
        
        # ============================================
        # POSICIONAR LABEL DE STATUS (CENTRADO ABAJO)
        # ============================================
        
        label_width = 500
        label_height = 60
        label_x = center_x - label_width // 2
        label_y = self.height() - 100 if self.height() > 100 else 600
        
        self.label.setGeometry(label_x, label_y, label_width, label_height)

    def resizeEvent(self, event):
        """Reposicionar elementos cuando cambie el tamaño"""
        super().resizeEvent(event)
        self.position_all_elements()

    def button_clicked_gestures(self):
        """Manejar el botón de control por gestos"""
        if not GESTURES_AVAILABLE:
            self.label.setText("❌ Control por gestos no disponible")
            return
            
        if not self.gesture_active:
            self.start_gesture_control()
        else:
            self.stop_gesture_control()
    
    def start_gesture_control(self):
        """Iniciar el control por gestos"""
        try:
            self.gesture_detector = QtHandGestureDetector()
            self.gesture_detector.gesture_detected.connect(self.parent_window.on_gesture_detected)
            self.gesture_detector.dance_mode_changed.connect(self.parent_window.on_dance_mode_changed)
            self.gesture_detector.status_updated.connect(self.parent_window.on_gesture_status_updated)
            
            if self.gesture_detector.start_detection():
                self.gesture_active = True
                self.gesture_button.setText("👋\nDETENER")
                self.gesture_button.setStyleSheet("""
                    QPushButton {
                        background-color: rgba(231, 76, 60, 0.85);
                        color: white;
                        border: 3px solid rgba(255, 255, 255, 0.8);
                        border-radius: 50px;
                        font-size: 16px;
                        font-weight: bold;
                        text-shadow: 2px 2px 4px rgba(0, 0, 0, 0.5);
                    }
                    QPushButton:hover {
                        background-color: rgba(231, 76, 60, 0.95);
                        border: 3px solid white;
                        transform: scale(1.08);
                        box-shadow: 0 8px 20px rgba(0, 0, 0, 0.3);
                    }
                    QPushButton:pressed {
                        background-color: rgba(192, 57, 43, 1.0);
                        transform: scale(1.02);
                    }
                """)
                self.label.setText("🤖 Control por gestos ACTIVO")
        except Exception as e:
            self.label.setText(f"❌ Error iniciando gestos: {str(e)}")
    
    def stop_gesture_control(self):
        """Detener el control por gestos"""
        try:
            self.gesture_active = False
            if self.gesture_detector:
                self.gesture_detector.stop_detection()
                self.gesture_detector = None
            
            self.gesture_button.setText("👋\nGESTOS")
            self.gesture_button.setStyleSheet("""
                QPushButton {
                    background-color: rgba(142, 68, 173, 0.85);
                    color: white;
                    border: 3px solid rgba(255, 255, 255, 0.8);
                    border-radius: 50px;
                    font-size: 16px;
                    font-weight: bold;
                    text-shadow: 2px 2px 4px rgba(0, 0, 0, 0.5);
                }
                QPushButton:hover {
                    background-color: rgba(142, 68, 173, 0.95);
                    border: 3px solid white;
                    transform: scale(1.08);
                    box-shadow: 0 8px 20px rgba(0, 0, 0, 0.3);
                }
                QPushButton:pressed {
                    background-color: rgba(125, 60, 152, 1.0);
                    transform: scale(1.02);
                }
            """)
            self.label.setText("⏹️ Control por gestos detenido")
        except Exception as e:
            self.label.setText(f"❌ Error deteniendo gestos: {str(e)}")


class MyWindow(QMainWindow):
    def __init__(self):
        super(MyWindow, self).__init__()
        
        print("🚀 Iniciando aplicación ARIS...")
        
        # Inicializar ROS 
        rclpy.init(args=None)
        self.node = rclpy.create_node('ros_gui')
        self.pub = self.node.create_publisher(String, '/chatter', 10)
        
        # Inicializar el asistente de voz 
        if AUDIO_AVAILABLE:
            self.assistant = VoiceAssistantBackend(status_callback=self.update_voice_status)
            # Iniciar el mensaje de bienvenida
            self.assistant.start_welcome_message()
            
        else:
            self.assistant = None
        
        # Inicializar timer ANTES de initUI
        self.inactivity_timer = QTimer()
        self.inactivity_timeout = 30000  # 30 segundos
        
        # Configurar UI
        self.initUI()
        
        # Configurar timer DESPUÉS de initUI
        self.setup_timers()
        
        print("✅ Aplicación ARIS iniciada correctamente")
        
    def setup_timers(self):
        """Configurar temporizadores"""
        # Conectar timer a función show_animated_face
        self.inactivity_timer.timeout.connect(self.show_animated_face)
        
    def initUI(self):
        print("🔧 Configurando interfaz...")
        
        self.setGeometry(100, 100, 800, 750)
        self.setWindowTitle("ARIS - Asistente Robótico")
        
        # Crear widget apilado
        self.stacked_widget = QStackedWidget()
        self.setCentralWidget(self.stacked_widget)
        
        # Crear widgets
        print("🔧 Creando widget de cara animada...")
        self.animated_face = AnimatedFaceWidget()
        
        print("🔧 Creando widget de interfaz principal...")
        self.main_interface = MainInterfaceWidget(self)
        
        # Conectar señal de la cara
        print("🔗 Conectando señales...")
        self.animated_face.face_clicked.connect(self.debug_face_clicked)
        self.animated_face.face_clicked.connect(self.show_main_interface)
        
        # Agregar widgets al stack
        self.stacked_widget.addWidget(self.animated_face)
        self.stacked_widget.addWidget(self.main_interface)
        
        # Mostrar cara inicialmente
        self.show_animated_face()
        
        # Pantalla completa
        self.showMaximized()
        
        print("✅ Interfaz configurada")
    
    def debug_face_clicked(self):
        """Método de debugging para verificar que la señal llega"""
        print("🚀 ¡ÉXITO! Señal face_clicked recibida en MyWindow!")
        print("📍 Ejecutando transición a interfaz principal...")
    
    def reset_inactivity_timer(self):
        """Reiniciar timer de inactividad"""
        try:
            if hasattr(self, 'inactivity_timer') and self.inactivity_timer:
                self.inactivity_timer.stop()
                self.inactivity_timer.start(self.inactivity_timeout)
        except Exception as e:
            print(f"Error reseteando timer: {e}")
    
    def show_animated_face(self):
        """Mostrar la cara animada"""
        try:
            print("🔄 Mostrando cara animada...")
            self.stacked_widget.setCurrentWidget(self.animated_face)
            if hasattr(self, 'inactivity_timer'):
                self.inactivity_timer.stop()
            
            # Detener gestos si están activos
            if (hasattr(self, 'main_interface') and 
                hasattr(self.main_interface, 'gesture_active') and 
                self.main_interface.gesture_active):
                self.main_interface.stop_gesture_control()
                
            print("✅ Cara animada mostrada")
        except Exception as e:
            print(f"Error mostrando cara: {e}")
    
    def show_main_interface(self):
        """Mostrar la interfaz principal"""
        try:
            print("🎉 Cambiando a interfaz principal...")
            
            # Verificar que main_interface existe
            if not hasattr(self, 'main_interface'):
                print("❌ Error: main_interface no está inicializado")
                return
                
            self.stacked_widget.setCurrentWidget(self.main_interface)
            self.reset_inactivity_timer()
            self.main_interface.label.setText("🤖 ¡Bienvenido! Selecciona una opción")
            
            print("✅ Interfaz principal mostrada")
        except Exception as e:
            print(f"❌ Error mostrando interfaz: {e}")
            import traceback
            traceback.print_exc()
    
    def button_clicked(self, point_name, command):
        """Manejar clics de botones"""
        print(f"🎯 Botón {point_name} clickeado")
        self.main_interface.label.setText(f"🚀 Navegando al punto {point_name}")
        self.send_position(command)
        self.reset_inactivity_timer()
        
    def button_clicked_voice(self):
        """Manejar botón de voz"""
        if not AUDIO_AVAILABLE or not self.assistant:
            self.main_interface.label.setText("❌ Asistente de voz no disponible")
            return
            
        if not hasattr(self, 'main_interface'):
            return
            
        if not self.assistant.is_listening:
            self.main_interface.voice_button.setEnabled(False)
            self.main_interface.voice_button.setText("🎤\nEscuchando...")
            self.main_interface.label.setText("🎤 Asistente activado")
            
            self.assistant.stop_welcome_message()
            self.assistant.start_listening()
        
        self.reset_inactivity_timer()

    @pyqtSlot(int, str)
    def on_gesture_detected(self, finger_count, command):
        """Slot para gestos detectados"""
        if not hasattr(self, 'main_interface'):
            return
            
        if command == "dance":
            self.main_interface.label.setText(f"🕺 MODO BAILE: {finger_count} dedos")
        else:
            self.main_interface.label.setText(f"✋ Gesto: {finger_count} dedos → Comando {command}")
            self.send_position(command)
        self.reset_inactivity_timer()

    @pyqtSlot(bool)
    def on_dance_mode_changed(self, activate):
        """Slot para modo baile"""
        if not hasattr(self, 'main_interface'):
            return
            
        if activate:
            self.main_interface.label.setText("🕺 ROBOT INICIANDO BAILE 🕺")
        else:
            self.main_interface.label.setText("🕺 Baile terminado")
        self.reset_inactivity_timer()

    @pyqtSlot(str, bool)
    def on_gesture_status_updated(self, message, is_error):
        """Slot para estado de gestos"""
        if not hasattr(self, 'main_interface'):
            return
            
        if not is_error and "dedos" not in message:
            self.main_interface.label.setText(f"🤖 Gestos: {message}")
        self.reset_inactivity_timer()
            
    def update_voice_status(self, message, is_error=False):
        """Actualizar estado de voz (igual que tu código original)"""
        if not hasattr(self, 'main_interface'):
            return
            
        self.main_interface.label.setText(message)
        
        if message == "Listo para escuchar":
            self.main_interface.voice_button.setEnabled(True)
            self.main_interface.voice_button.setText("🎤\nHABLAR")
        
        self.reset_inactivity_timer()

    def send_position(self, position):
        """Enviar posición (igual que tu código original)"""
        print(f"📍 ENVIANDO POSICIÓN: {position}")
        vel1 = String()
        vel1.data = position
        self.pub.publish(vel1)

    def closeEvent(self, event):
        """Cerrar aplicación (igual que tu código original)"""
        print("🔄 Cerrando aplicación...")
        
        if hasattr(self.main_interface, 'gesture_active') and self.main_interface.gesture_active:
            self.main_interface.stop_gesture_control()
            
        if hasattr(self, 'assistant') and self.assistant:
            self.assistant.stop_welcome_message()
            
        self.node.destroy_node()
        rclpy.shutdown()
        
        print("✅ Aplicación cerrada")
        event.accept()


def main():
    print("🚀 Iniciando aplicación ARIS...")
    
    app = QApplication(sys.argv)
    app.setApplicationName("ARIS - Asistente Robótico")
    app.setOrganizationName("ARIS Project")
    
    try:
        win = MyWindow()
        win.show()
        
        print("🎉 Aplicación lista - ventana mostrada")
        sys.exit(app.exec_())
        
    except Exception as e:
        print(f"❌ Error crítico: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()