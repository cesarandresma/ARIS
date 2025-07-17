import os
import threading
import time
import pyaudio
from vosk import Model, KaldiRecognizer
import json
import tempfile
import requests
from gtts import gTTS
import google.generativeai as genai

class VoiceAssistantBackend:
    
    def __init__(self, status_callback=None, audio_dir="/home/ister/turtlebot4_ws/src/turtlebot4_python/turtlebot4_python/audios",
                 model_path="/home/ister/turtlebot4_ws/src/turtlebot4_python/turtlebot4_python/vosk-model-small-es-0.42"):
        """
        Inicializar el backend del asistente de voz con VOSK para reconocimiento offline.
        
        Args:
            status_callback: Función para actualizar el estado en la UI
            audio_dir: Directorio donde se encuentran los archivos de audio
            model_path: Ruta al modelo de VOSK (debe ser descargado previamente)
        """
        # Verificar si el modelo existe
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Modelo VOSK no encontrado en {model_path}.")
        
        # Inicializar el modelo VOSK
        self.model = Model(model_path)
        
        # Configuración de audio
        self.sample_rate = 16000
        
        # Status callback for UI updates
        self.status_callback = status_callback
        
        # State variables
        self.is_listening = False
        self.is_welcome_message_running = False
        self.welcome_thread = None
        self.should_stop_listening = False
        
        # Audio directory
        self.audio_dir = audio_dir
        
        # PyAudio
        self.audio = pyaudio.PyAudio()
        
        # Configurar Gemini AI
        self.setup_api()
        
        # Cache para verificación de internet
        self.last_internet_check = 0
        self.internet_available = False
        self.internet_check_interval = 30  # Verificar cada 30 segundos

    def setup_api(self):
        """Inicializar configuraciones de API"""
        try:
            GOOGLE_API_KEY = "AIzaSyC_yDiL70NJSxp80zecGXBx3UnrRYfIApk"
            genai.configure(api_key=GOOGLE_API_KEY)
            self.model_gemini = genai.GenerativeModel(
                model_name='gemini-1.5-flash',
                system_instruction='Eres un asistente virtual llamado Aris del Instituto Universitario Rumiñahui. '
                                  'Siempre que una pregunta esté relacionada con alguna entidad o acrónimos relaciónalo con el sistema educativo de Ecuador; '
                                  'caso contrario, puedes responder de manera general usando el modelo base de forma rápida y precisa. '
                                  'Mantén tus respuestas concisas y claras, máximo 2 párrafos.'
            )
            self.gemini_available = True
            self.update_status("Gemini AI configurado correctamente")
        except Exception as e:
            self.gemini_available = False
            self.update_status(f"Error configurando Gemini AI: {str(e)}", is_error=True)

    def check_internet_connection(self):
        """Verificar si hay conexión a internet"""
        current_time = time.time()
        
        # Usar cache si la verificación es reciente
        if current_time - self.last_internet_check < self.internet_check_interval:
            return self.internet_available
        
        try:
            # Intentar conectar a Google DNS
            response = requests.get("http://8.8.8.8", timeout=3)
            self.internet_available = True
            self.last_internet_check = current_time
            return True
        except:
            try:
                # Intentar conectar a un servidor alternativo
                response = requests.get("http://1.1.1.1", timeout=3)
                self.internet_available = True
                self.last_internet_check = current_time
                return True
            except:
                self.internet_available = False
                self.last_internet_check = current_time
                return False

    def get_gemini_response(self, query):
        """Obtener respuesta de Gemini AI"""
        try:
            if not self.gemini_available:
                return None
                
            self.update_status("Consultando con Gemini AI...")
            response = self.model_gemini.generate_content(query)
            
            if response and response.text:
                return response.text.strip()
            else:
                return None
                
        except Exception as e:
            self.update_status(f"Error con Gemini AI: {str(e)}", is_error=True)
            return None

    def text_to_speech_gemini_response(self, text):
        """Convertir respuesta de Gemini a audio y reproducir"""
        try:
            # Crear mensaje de Aris
            message = f"Hola, {text}"
            
            # Generar audio con gTTS
            tts = gTTS(text=message, lang="es", slow=False)
            
            # Usar archivo temporal
            with tempfile.NamedTemporaryFile(delete=False, suffix='.mp3') as temp_file:
                temp_filename = temp_file.name
                tts.save(temp_filename)
            
            # Reproducir el archivo
            os.system(f'mpg123 {temp_filename}')
            
            # Limpiar archivo temporal
            try:
                os.unlink(temp_filename)
            except:
                pass
                
            return True
            
        except Exception as e:
            self.update_status(f"Error generando audio de respuesta: {str(e)}", is_error=True)
            return False

    def update_status(self, message, is_error=False):
        """Update status"""
        if self.status_callback:
            self.status_callback(message, is_error)
        print(message)  # Consola output

    def listen_and_transcribe(self):
        """
        Escuchar micrófono y transcribir texto usando VOSK (offline).
        
        Return:
            str: Texto o vacío si hay error
        """
        self.update_status("Escuchando...")
        
        try:
            # Crear reconocedor
            recognizer = KaldiRecognizer(self.model, self.sample_rate)
            
            # Abrir stream de micrófono
            stream = self.audio.open(format=pyaudio.paInt16, 
                                    channels=1, 
                                    rate=self.sample_rate,
                                    input=True, 
                                    frames_per_buffer=4000)
            
            stream.start_stream()
            
            # Variable para determinar cuando el usuario ha terminado de hablar
            silence_counter = 0
            listening_started = False
            final_result = ""
            
            # Reiniciar bandera de detención
            self.should_stop_listening = False
            
            self.update_status("Esperando voz...")
            
            # Bucle principal de escucha
            while not self.should_stop_listening:
                data = stream.read(4000, exception_on_overflow=False)
                
                # Si VOSK detecta audio válido
                if recognizer.AcceptWaveform(data):
                    result = json.loads(recognizer.Result())
                    if result.get("text", "").strip():
                        listening_started = True
                        final_result = result["text"]
                        self.update_status(f"Detectado: {final_result}")
                        
                # Procesar resultados parciales para detectar silencio
                partial = json.loads(recognizer.PartialResult())
                
                # Si ya empezó a escuchar y hay silencio (texto vacío)
                if listening_started and not partial.get("partial", "").strip():
                    silence_counter += 1
                    # Aproximadamente 2 segundos de silencio (depende del buffer)
                    if silence_counter > 10:  
                        break
                else:
                    silence_counter = 0
                
                # Tiempo máximo de escucha (15 segundos)
                if listening_started and silence_counter == 0:
                    # Incrementar contador para eventualmente terminar
                    # aunque el usuario siga hablando
                    silence_counter += 0.1
                    if silence_counter > 40:  # ~15 segundos
                        break
            
            # Cerrar stream
            stream.stop_stream()
            stream.close()
            
            return final_result.lower()
            
        except Exception as e:
            self.update_status(f"Error en reconocimiento de voz: {str(e)}", is_error=True)
            return ""
            
    def play_audio(self, filename):
        """
        Reproducir audio usando mpg123.
        
        """
        try:
            # Construct full path to the audio file
            full_path = os.path.join(self.audio_dir, filename)
            
            # Check if file exists
            if not os.path.exists(full_path):
                self.update_status(f"Archivo de audio no encontrado: {filename}", is_error=True)
                return False
                
            # Play the audio file
            os.system(f'mpg123 {full_path}')
            return True
            
        except Exception as e:
            self.update_status(f"Error reproduciendo audio: {str(e)}", is_error=True)
            return False

    def process_input(self, text):
        """
        Procesamiento de entrada de voz y reproducir el audio.
        Ahora incluye integración con Gemini AI para respuestas inteligentes.
        """
        text = text.lower()
        
        # Verificar comandos predefinidos primero
        if any(keyword in text for keyword in ["hola", "buenos días", "buenas tardes"]):
            self.play_audio('hola.mp3')
            return "Reproduciendo saludo"
            
        elif any(keyword in text for keyword in ["autoridades", "autoridad", "dueños", "promotores", "mentores", "promotor", "mentor"]):
            self.play_audio('autoridades.mp3')
            return "Reproduciendo información de autoridades"
        
        elif any(keyword in text for keyword in ["ubicación", "dirección", "encuentra", "queda"]):
            self.play_audio('ubicacion.mp3')
            return "Reproduciendo ubicacion"
            
        elif any(keyword in text for keyword in ["información", "carreras", "un formación", "mi formación", "in formación", "en formación", "y formación"]):
            self.play_audio('informacion.mp3')
            return "Reproduciendo informacion"

        elif any(keyword in text for keyword in ["requisitos", "documentos", "ingresos"]):
            self.play_audio('requisitos.mp3')
            return "Reproduciendo requisitos"
        
        elif any(keyword in text for keyword in ["maestria", "maestrias"]):
            self.play_audio('maestria.mp3')
            return "Reproduciendo maestria"
        
        elif any(keyword in text for keyword in ["tecnología superior"]):
            self.play_audio('superior.mp3')
            return "Reproduciendo superior"
        
        elif any(keyword in text for keyword in ["tecnicatura", "tecnicaturas"]):
            self.play_audio('tecnicatura.mp3')
            return "Reproduciendo tecnicatura"
        
        elif any(keyword in text for keyword in ["cases", "case"]):
            self.play_audio('caces.mp3')
            return "Reproduciendo caces"
        
        elif "salir" in text:
            #self.is_welcome_message_running = True
            time.sleep(1)
            self.play_audio("adios.mp3")
            self.is_welcome_message_running = True
            self.start_welcome_message()
            return "Reproduciendo despedida"
        
        else:
            # Si no encuentra palabras clave, intentar usar Gemini AI
            return self.handle_unknown_command(text)

    def handle_unknown_command(self, text):
        """
        Manejar comandos no reconocidos usando Gemini AI si hay internet,
        o reproducir audio de fallback si no hay internet.
        """
        # Verificar conexión a internet
        if self.check_internet_connection():
            self.update_status("Procesando consulta con inteligencia artificial...")
            
            # Obtener respuesta de Gemini
            gemini_response = self.get_gemini_response(text)
            
            if gemini_response:
                # Convertir respuesta a audio y reproducir
                if self.text_to_speech_gemini_response(gemini_response):
                    return f"Respuesta de IA: {gemini_response[:50]}..."
                else:
                    # Si falla la conversión a audio, usar audio de fallback
                    self.play_audio("hola.mp3")
                    return "Error generando respuesta de audio, reproduciendo saludo"
            else:
                # Si Gemini no puede responder, usar audio de fallback
                self.play_audio("hola.mp3")
                return "No pude generar respuesta, reproduciendo saludo"
        else:
            # Sin internet, usar audio de fallback
            self.update_status("Sin conexión a internet, usando respuesta predeterminada")
            self.play_audio("hola.mp3")
            return "Sin internet - Reproduciendo saludo predeterminado"

    def start_listening(self):
        
        if self.is_listening:
            return False
            
        self.is_listening = True
        self.is_welcome_message_running = False
        
        # Stop welcome message if it's running
        if self.welcome_thread and self.welcome_thread.is_alive():
            self.is_welcome_message_running = False
            self.welcome_thread.join(1)  
            
        # Start listening thread
        threading.Thread(target=self.listen_and_respond, daemon=True).start()
        return True

    def listen_and_respond(self):
        try:
            text = self.listen_and_transcribe()
            if text:
                self.update_status(f"Texto detectado: {text}")
                status = self.process_input(text)
                self.update_status(status)
                return status
            return ""
        finally:
            self.is_listening = False
            self.update_status("Listo para escuchar")

    def start_welcome_message(self):
        
        if not self.is_welcome_message_running:
            self.is_welcome_message_running = True
            self.welcome_thread = threading.Thread(target=self.play_welcome_message, daemon=True)
            self.welcome_thread.start()

    def stop_welcome_message(self):
        
        self.is_welcome_message_running = False
        if self.welcome_thread and self.welcome_thread.is_alive():
            self.welcome_thread.join(1)

    def play_welcome_message(self):
        
        while self.is_welcome_message_running:
            try:
                self.play_audio("bienvenida.mp3")
                time.sleep(20)  # Wait before repeating
            except Exception as e:
                self.update_status(f"Error en mensaje de bienvenida: {str(e)}", is_error=True)
                break
    
    def cleanup(self):
        """Liberar recursos"""
        self.should_stop_listening = True
        self.stop_welcome_message()
        self.audio.terminate()

# Example usage
if __name__ == "__main__":
    # Test the backend functionality
    def print_status(msg, is_error=False):
        prefix = "ERROR: " if is_error else "STATUS: "
        print(prefix + msg)
    
    try:
        # Crear instancia del asistente
        assistant = VoiceAssistantBackend(
            status_callback=print_status,
            model_path="vosk-model-small-es-0.42"
        )
        
        # Start welcome message
        assistant.start_welcome_message()
        
        # Wait for 5 seconds, then simulate button press
        #time.sleep(5)
        
        # Stop welcome and start listening
        #assistant.stop_welcome_message()
        #assistant.start_listening()
        
        # Mantener el programa en ejecución
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Programa terminado por el usuario")
    
    except Exception as e:
        print(f"Error en la aplicación: {e}")
    
    finally:
        # Cleanup
        if 'assistant' in locals():
            assistant.cleanup()
