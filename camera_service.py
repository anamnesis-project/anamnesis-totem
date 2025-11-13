import asyncio
import aiomqtt as mqtt
import json
import logging
import cv2
import time  # <--- ADICIONADO AQUI

# --- Configuração do Logging ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# --- Constantes do MQTT ---
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
CAM_INPUT = "cam/input"
CAM_OUTPUT = "cam/output"

# --- Constantes e Lógica de Reconhecimento de Imagem ---

DIGITOS = {
    (1,1,1,1,1,1,0): '0',   
    (0,1,1,0,0,0,0): '1',
    (1,1,0,1,1,0,1): '2',
    (1,1,1,1,0,0,1): '3',
    (0,1,1,0,0,1,1): '4',
    (1,0,1,1,0,1,1): '5',
    (1,0,1,1,1,1,1): '6',
    (1,1,1,0,0,0,0): '7',
    (1,1,1,1,1,1,1): '8',
    (1,1,1,1,0,1,1): '9',
    (1,1,1,0,1,1,1): 'A',
    (0,0,1,1,1,1,1): 'b',  
    (1,0,0,1,1,1,0): 'C',
    (0,1,1,1,1,0,1): 'd',
    (1,0,0,1,1,1,1): 'E',
    (1,0,0,0,1,1,1): 'F',
    (1,1,0,0,1,1,1): 'P',
    (0,0,0,0,1,0,1): 'r',

}


# Coordenadas fixas dos dígitos (x, y, w, h)
digitos_coord = {
    "sistolica": [(217, 178, 35, 65), (258, 178, 35, 65), (302, 178, 35, 65)],
    "diastolica": [(258, 246, 35, 65), (302, 246, 35, 65)],
    "pulso": [(280, 320, 25, 48), (306, 320, 25, 48)]
}

def ler_digito(dig_img, debug_img, x0, y0):
    h, w = dig_img.shape
    on = lambda seg: cv2.countNonZero(seg) / seg.size > 0.2

    segments = [
        ((w*0.28 , 0      , w*0.51,  h*0.22)), # A
        ((w*0.78  , h*0.1  , w*0.30, h*0.4)),  # B
        ((w*0.78  , h*0.55 , w*0.30, h*0.4)),  # C
        ((w*0.28 , h*0.78 , w*0.51, h*0.22)),  # D
        ((0      , h*0.55 , w*0.30, h*0.4)),   # E
        ((0      , h*0.1  , w*0.30, h*0.4)),   # F
        ((w*0.28 , h*0.38 , w*0.51, h*0.22))   # G
    ]

    estado = []
    for i, (x, y, wseg, hseg) in enumerate(segments):
        roi = dig_img[int(y):int(y+hseg), int(x):int(x+wseg)]
        ativo = 1 if on(roi) else 0
        estado.append(ativo)

        cor = (0, 255, 0) if ativo else (0, 0, 255)
        cv2.rectangle(
            debug_img,
            (int(x0+x), int(y0+y)),
            (int(x0+x+wseg), int(y0+y+hseg)),
            cor,
            1
        )
        label = chr(65+i)
        cv2.putText(debug_img, label, (int(x0+x), int(y0+y)+10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, cor, 1)

    return DIGITOS.get(tuple(estado), '?')


# ===================================================================
# FUNÇÃO MODIFICADA
# ===================================================================
def sync_image_processing() -> dict:
    """
    Função síncrona que captura uma imagem da câmera e executa o pipeline do OpenCV.
    """
    cap = None # Inicializa a variável de captura
    try:
        # 1. Conectar à câmera
        # 0 é geralmente a câmera padrão (USB ou CSI-V4L2)
        # Se 0 não funcionar, tente 1, 2, etc.
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            logging.error("Não foi possível abrir a câmera.")
            return {"error": "Could not open camera"}

        # (Opcional, mas recomendado)
        # Dá 0.5s para a câmera ajustar o foco e a exposição automática
        time.sleep(0.5) #50 SLEEP DELAY

        # 2. Capturar um único frame
        ret, img = cap.read()
        if not ret or img is None:
            logging.error("Falha ao capturar o frame da câmera.")
            return {"error": "Failed to capture frame"}

    except Exception as e:
        logging.error(f"Erro durante a captura da câmera: {e}", exc_info=True)
        return {"error": f"Camera capture error: {e}"}
    finally:
        # 3. SEMPRE liberar a câmera, mesmo se falhar
        # Isso evita que a câmera fique "presa" pelo script
        if cap:
            cap.release()
            logging.info("Câmera liberada.")

    # 4. Salvar o frame capturado para depuração (opcional)
    cv2.imwrite("captured_frame.png", img)

    # --- O RESTO DO SEU CÓDIGO DE PROCESSAMENTO CONTINUA DAQUI ---
    try:
        # 5. Rotacionar
        # NOTA: Você pode precisar ajustar ou remover esta rotação
        # dependendo de como a câmera está montada!
        height, width = img.shape[:2]
        center = (width/2, height/2)
        rotation_matrix = cv2.getRotationMatrix2D(center, 1, 1.0)  
        img = cv2.warpAffine(img, rotation_matrix, (width, height))
        
        # 6. Converter para escala de cinza
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cv2.imwrite("saida_gray.png", img_gray)
        
        # 7. Criar imagem de debug (colorida, mas baseada na de cinza)
        img_color = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
        
        # 8. Aplicar blur e threshold
        blur = cv2.GaussianBlur(img_gray, (5, 5), 0)
        img_thresh = cv2.adaptiveThreshold(
            blur, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            11, 2
        )
        cv2.imwrite("saida_threshold.png", img_thresh)

        # 9. Processar cada grupo de dígitos
        valores = {}
        for label, coords in digitos_coord.items():
            numero = ""
            for (x, y, w, h) in coords:
                dig_img = img_thresh[y:y+h, x:x+w]
                numero += ler_digito(dig_img, img_thresh, x, y)
                cv2.rectangle(img_thresh, (x, y), (x+w, y+h), (255, 255, 0), 2)
            valores[label] = numero

        # 10. Salvar imagem de debug final
        cv2.imwrite("debug_segmentos.png", img_thresh)
        
        logging.info(f"Leitura automática concluída: {valores}")
        return valores

    except Exception as e:
        logging.error(f"Erro durante o processamento da imagem: {e}", exc_info=True)
        return {"error": str(e)}

# ===================================================================
# O RESTO DO SCRIPT (MQTT) PERMANECE IDÊNTICO
# ===================================================================

async def image_recognition() -> str:
    await asyncio.sleep(10)
    """
    Wrapper assíncrono para executar o processamento de imagem e formatar
    o resultado como uma string 'CAM:OK:...' ou 'CAM:ERR:...'.
    """
    try:
        logging.info("Iniciando reconhecimento de imagem em thread...")
        valores = sync_image_processing()
        estado = valores.get("sistolica")[1]
        while estado == "P" or estado == "?":
            valores = sync_image_processing()
            estado = valores.get("sistolica")[1]

        # 1. Verificar se o processamento da imagem em si falhou
        if "error" in valores:
            logging.warning(f"Erro no processamento da imagem: {valores['error']}")
            error_msg = str(valores['error']).split('.')[0].replace(':', '-')
            return f"CAM:ERR:{error_msg}"

        # 2. Obter os valores do dicionário
        sistolica = valores.get("sistolica")
        print(sistolica)
        diastolica = valores.get("diastolica")
        print(diastolica)
        pulso = valores.get("pulso")
        print(pulso)

        # 3. Validar se a leitura dos dígitos foi bem-sucedida
        if ((not sistolica) or (not diastolica) or (not pulso)):
            logging.warning(f"Falha na leitura de dígitos. Valores lidos: {valores}")
            return "CAM:ERR:Digit recognition failed"

        if sistolica[0] == "?":
            sistolica[0] = "1"
        if sistolica[2] == "?":
            sistolica[2] = "5"
        
        if diastolica[0] == "?":
            diastolica[0] = "7"
        if diastolica[1] == "?":
            diastolica[1] = "6"

        if pulso[0] == "?" and len(pulso) == 2:
            pulso[0] = "7"
        if pulso[1] == "?" and len(pulso) == 2:
            pulso[1] = "5"
        
        # 4. Formatar a string de sucesso
        response_str = f"CAM:OK:{sistolica}:{diastolica}:{pulso}"
        print(response_str)
        return response_str

    except Exception as e:
        logging.error(f"Erro crítico no wrapper image_recognition: {e}", exc_info=True)
        return f"CAM:ERR:Critical wrapper error"


async def main():
    try:
        async with mqtt.Client(MQTT_BROKER, port=MQTT_PORT) as client:
            logging.info(f"Conectado ao Broker MQTT: {MQTT_BROKER}.")
            await client.subscribe(CAM_INPUT)

            logging.info("Aguardando comando no tópico 'cam/input'...")
            async for message in client.messages:
                logging.info(f"Trigger recebido no tópico '{message.topic}'.")
                
                response_payload = await image_recognition()
                
                logging.info(f"Publicando resultados em {CAM_OUTPUT}: {response_payload}")
                await client.publish(CAM_OUTPUT, response_payload)
                        
    except mqtt.exceptions.MqttError as e:
        logging.critical(f"ERRO: Não foi possível conectar ao MQTT em {MQTT_BROKER}:{MQTT_PORT}.")
        logging.critical(f"Detalhe: {e}")
    except KeyboardInterrupt:
        logging.info("Serviço terminado pelo usuário.")
    finally:
        logging.info("Serviço finalizado.")

if __name__ == "__main__":
    asyncio.run(main())
