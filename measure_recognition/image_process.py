import cv2
import pytesseract
import re

# Se estiver no Windows, ajuste o caminho para o executável do Tesseract:
# pytesseract.pytesseract.tesseract_cmd = r"C:\Program Files\Tesseract-OCR\tesseract.exe"

# === 1. Carrega a imagem ===
img = cv2.imread('esfigmo.png')  # troque para esfigmo2.png se necessário
if img is None:
    raise FileNotFoundError("Imagem 'esfigmo.png' não encontrada.")

# Guardar dimensões para debug
h, w = img.shape[:2]
print(f"Dimensões da imagem: {w} x {h}")

# === 2. Pré-processamento ===
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray, (3, 3), 0)
_, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

# === 3. Definição das ROIs (coordenadas fixas) ===
# ATENÇÃO: (y1:y2, x1:x2) -> OpenCV usa (linha, coluna)
# Coordenadas para imagem de 129x175 px (ajuste se a resolução for diferente)
sys_y1, sys_y2, sys_x1, sys_x2 = 25, 55, 75, 130   # SYS: parte superior (108)
dia_y1, dia_y2, dia_x1, dia_x2 = 55, 85, 75, 130   # DIA: meio (62)
pul_y1, pul_y2, pul_x1, pul_x2 = 85, 115,75, 130  # PUL: inferior (77)

# Extrai as regiões binarizadas
sys_roi = thresh[sys_y1:sys_y2, sys_x1:sys_x2]
dia_roi = thresh[dia_y1:dia_y2, dia_x1:dia_x2]
pul_roi = thresh[pul_y1:pul_y2, pul_x1:pul_x2]

# === 4. Função OCR auxiliar ===
def extract_digits(roi):
    custom_config = r'--oem 3 --psm 7 -c tessedit_char_whitelist=0123456789'
    text = pytesseract.image_to_string(roi, config=custom_config)
    text = re.sub(r'\D', '', text)  # remove não numéricos
    return text

sys_val = extract_digits(sys_roi)
dia_val = extract_digits(dia_roi)
pul_val = extract_digits(pul_roi)

print(f"SYS: {sys_val}")
print(f"DIA: {dia_val}")
print(f"PUL: {pul_val}")

# === 5. Visualização para depuração ===

# Desenha retângulos nas regiões
img_debug = img.copy()
cv2.rectangle(img_debug, (sys_x1, sys_y1), (sys_x2, sys_y2), (0, 0, 255), 2)   # Vermelho SYS
cv2.rectangle(img_debug, (dia_x1, dia_y1), (dia_x2, dia_y2), (0, 255, 0), 2)   # Verde DIA
cv2.rectangle(img_debug, (pul_x1, pul_y1), (pul_x2, pul_y2), (255, 0, 0), 2)   # Azul PUL

# Mostra janelas de debug
cv2.imshow("Imagem Original + ROIs", img_debug)
cv2.imshow("SYS ROI", sys_roi)
cv2.imshow("DIA ROI", dia_roi)
cv2.imshow("PUL ROI", pul_roi)
cv2.waitKey(0)
cv2.destroyAllWindows()
