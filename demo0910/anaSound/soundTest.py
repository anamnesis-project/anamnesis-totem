import subprocess
import os
import time

# --- Configurações ---
DURACAO_GRAVACAO = 5  # Segundos
ARQUIVO_DE_AUDIO = "gravacao_temporaria.wav"

# Altere 'plughw:1,0' para o identificador do seu microfone USB (visto com 'arecord -l')
# Por exemplo: 'plughw:1,0', 'hw:1,0', etc.
NOME_DO_DISPOSITIVO_DE_GRAVACAO = 'plughw:3,0'

def gravar_audio():
    """Grava áudio do microfone por um tempo determinado."""
    print(f"Gravando por {DURACAO_GRAVACAO} segundos...")
    
    # Comando para gravar usando arecord
    comando_gravar = [
        'arecord',
        '-D', NOME_DO_DISPOSITIVO_DE_GRAVACAO, # Dispositivo de captura
        '-f', 'cd',                            # Qualidade de CD (16-bit, 44100 Hz)
        '-d', str(DURACAO_GRAVACAO),           # Duração em segundos
        ARQUIVO_DE_AUDIO                       # Nome do arquivo de saída
    ]
    
    try:
        # Executa o comando e espera ele terminar
        subprocess.run(comando_gravar, check=True, stderr=subprocess.PIPE)
        print("Gravação concluída.")
        return True
    except subprocess.CalledProcessError as e:
        print("ERRO ao gravar o áudio. Verifique o nome do dispositivo.")
        print(f"Detalhes do erro: {e.stderr.decode()}")
        return False
    except FileNotFoundError:
        print("ERRO: O comando 'arecord' não foi encontrado. Verifique se 'alsa-utils' está instalado.")
        return False

def tocar_audio():
    """Toca o arquivo de áudio gravado."""
    if not os.path.exists(ARQUIVO_DE_AUDIO):
        print(f"Arquivo {ARQUIVO_DE_AUDIO} não encontrado para tocar.")
        return

    print("Tocando o áudio gravado...")
    
    # Comando para tocar usando aplay
    comando_tocar = ['aplay', ARQUIVO_DE_AUDIO]
    
    try:
        # Executa o comando
        subprocess.run(comando_tocar, check=True, stderr=subprocess.PIPE)
        print("Reprodução concluída.")
    except subprocess.CalledProcessError as e:
        print("ERRO ao tocar o áudio.")
        print(f"Detalhes do erro: {e.stderr.decode()}")
    except FileNotFoundError:
        print("ERRO: O comando 'aplay' não foi encontrado. Verifique se 'alsa-utils' está instalado.")


def limpar_arquivo():
    """Remove o arquivo de áudio temporário."""
    if os.path.exists(ARQUIVO_DE_AUDIO):
        os.remove(ARQUIVO_DE_AUDIO)
        print(f"Arquivo temporário '{ARQUIVO_DE_AUDIO}' removido.")

if __name__ == "__main__":
    try:
        if gravar_audio():
            # Pequena pausa para garantir que o arquivo foi salvo corretamente
            time.sleep(0.5) 
            tocar_audio()
    finally:
        # Garante que o arquivo temporário seja sempre removido
        limpar_arquivo()
