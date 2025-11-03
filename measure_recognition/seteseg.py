import cv2

img = cv2.imread("2025-11-01-130816.jpg")
height, width = img.shape[:2]
center = (width/2, height/2)

# Criar matriz de rotação
rotation_matrix = cv2.getRotationMatrix2D(center, 3, 1.0)  
img = cv2.warpAffine(img, rotation_matrix, (width, height))
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imwrite("saida_gray.png", img)
img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
blur = cv2.GaussianBlur(img, (11, 11), 0)
img = cv2.adaptiveThreshold(
    blur, 255,
    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
    cv2.THRESH_BINARY_INV,
    11, 2
)
cv2.imwrite("saida_threshold.png", img)


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
    (1,0,0,0,1,1,1): 'F'
}

def ler_digito(dig_img, debug_img, x0, y0):
    h, w = dig_img.shape
    on = lambda seg: cv2.countNonZero(seg) / seg.size > 0.3

    segments = [
        ((w*0.2, 0, w*0.6, h*0.15)),       # A
        ((w*0.8, h*0.1, w*0.2, h*0.4)),    # B
        ((w*0.8, h*0.55, w*0.2, h*0.4)),   # C
        ((w*0.2, h*0.85, w*0.6, h*0.15)),  # D
        ((0, h*0.55, w*0.2, h*0.4)),       # E
        ((0, h*0.1, w*0.2, h*0.4)),        # F
        ((w*0.2, h*0.45, w*0.6, h*0.15))   # G
    ]

    estado = []
    for i, (x, y, wseg, hseg) in enumerate(segments):
        roi = dig_img[int(y):int(y+hseg), int(x):int(x+wseg)]
        ativo = 1 if on(roi) else 0
        estado.append(ativo)

        cor = (0, 255, 0) if ativo else (0, 0, 255)  # verde = aceso, vermelho = apagado
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

# --- Coordenadas fixas dos dígitos (x, y, w, h) ---
digitos_coord = {
    "sistolica": [(285, 48, 33, 60), (330, 48, 33, 60), (375, 47, 33, 60)],
    "diastolica": [(332, 118, 33, 60), (374, 116, 33, 60)],
    "pulso": [(353, 193, 22, 45), (382, 191, 22, 45)]
}

# --- Processar cada grupo ---
valores = {}
for label, coords in digitos_coord.items():
    numero = ""
    for (x, y, w, h) in coords:
        dig_img = img[y:y+h, x:x+w]
        numero += ler_digito(dig_img, img, x, y)
        cv2.rectangle(img, (x, y), (x+w, y+h), (255, 255, 0), 1)  # amarelo = caixa do dígito
    valores[label] = numero

print("\nLeitura automática:")
for k, v in valores.items():
    print(f"{k.capitalize()}: {v}")

cv2.imwrite("debug_segmentos.png", img)
print("\nImagem de depuração salva: 'debug_segmentos.png'")





