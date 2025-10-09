# -*- coding: utf-8 -*-

# =============================================================================
# --- Imports de Bibliotecas ---
# =============================================================================
from flask import Flask, render_template
from flask_socketio import SocketIO
import time
import threading
import serial  # Para comunicação com a porta serial
import json    # Para processar o formato de dados JSON

# =============================================================================
# --- Configuração Inicial ---
# =============================================================================
# Inicializa o aplicativo Flask
app = Flask(__name__)
app.config['SECRET_KEY'] = 'totem_secret_key'

# Inicializa o SocketIO para comunicação em tempo real
socketio = SocketIO(app)

# =============================================================================
# --- Conexão com a Porta Serial (ESP32) ---
# =============================================================================
# Tenta abrir a porta serial. Se falhar, o programa continua rodando
# e reporta o erro, em vez de quebrar.
try:
    # '/dev/ttyS0' é a porta serial de hardware nos pinos GPIO do Raspberry Pi
    ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)
    ser.flush()
    print("✅ Porta serial conectada com sucesso ao ESP.")
except serial.SerialException as e:
    ser = None
    print(f"⚠️  AVISO: Não foi possível conectar à porta serial ({e}). O servidor rodará sem dados do ESP.")

# =============================================================================
# --- Thread de Leitura de Dados do Hardware ---
# =============================================================================
def data_reader_thread():
    """
    Esta função roda em um loop infinito em segundo plano.
    A cada 2 segundos, ela comanda o ESP para fazer uma medição e
    retransmite o resultado para o tablet via WebSocket.
    """
    print("🚀 Iniciando thread de comunicação com o ESP...")
    while True:
        if ser:
            try:
                print("\n[Pi -> ESP] Enviando comando 'M'...")
                ser.write(b'M')
                
                print("[Pi <- ESP] Aguardando resposta...")
                response_line = ser.readline()

                if response_line:
                    print(f"[Pi <- ESP] Resposta Bruta Recebida: {response_line}")
                    data = json.loads(response_line.decode('utf-8').rstrip())
                    socketio.emit('sensor_update', data)
                    print(f"[Pi -> Tablet] Dados enviados para o tablet: {data}")
                else:
                    print("[Pi <- ESP] Nenhuma resposta recebida. Verificar ESP e fiação.")
                
            except (serial.SerialException, json.JSONDecodeError, UnicodeDecodeError) as e:
                print(f"!!! ERRO na comunicação ou processamento de dados: {e}")
        else:
            socketio.emit('sensor_update', {"error": "ESP desconectado"})
            
        socketio.sleep(2)

# =============================================================================
# --- Rotas do Servidor Web ---
# =============================================================================
@app.route('/')
def index():
    """
    Serve a página principal 'index.html' para o navegador do tablet.
    """
    return render_template('index.html')

# =============================================================================
# --- Eventos do WebSocket ---
# =============================================================================
thread = None

@socketio.on('connect')
def handle_connect():
    """
    Chamado quando um cliente (tablet) se conecta ao servidor.
    """
    global thread
    print('✅ Um tablet se conectou!')
    
    if thread is None or not thread.is_alive():
        thread = threading.Thread(target=data_reader_thread)
        thread.daemon = True
        thread.start()

@socketio.on('disconnect')
def handle_disconnect():
    """
    Chamado quando um cliente (tablet) se desconecta.
    """
    print('🔌 Um tablet se desconectou.')

# =============================================================================
# --- Inicialização do Servidor ---  <<< ESSA PARTE ESTAVA FALTANDO
# =============================================================================
if __name__ == '__main__':
    print("🚀 Iniciando servidor na rede local...")
    # O host '0.0.0.0' torna o servidor visível para outros dispositivos na rede
    socketio.run(app, host='0.0.0.0', port=5000)

