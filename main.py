import cv2, math, time, numpy, imutils, threading
from networktables import NetworkTablesInstance

    
KNOWN_DISTANCE = 118 # distância real do objeto para a câmera (só deve ser usada para calibrar a largura focal da câmera)
KNOWN_WIDTH = 55 # largura real do objeto (usada para calcular a distância da câmera para o objeto)
F = 10 # largura focal (calculada quando você sabe a sua distância, no artigo q eu mandei fala mais sobre isso)

STANDART_ERROR = 105 # erro padrão dos cálculos, calculamos por tentativa e erro

team = 6404 # número da equipe, pra configurar a comunicação com a Shuffleboard

dist = 0 # variável para armazenar a distância da câmera para o objeto

def process(frame): # método pra processar as imagens
    
    # O mínimo de cor pra filtrar, peguei por tentativa e erro
    low_H = 50
    low_S = 50
    low_V = 150

    # O máximo de cor pra filtrar, tbm peguei por tentativa e erro
    high_H = 100
    high_S = 200
    high_V = 255
    
    # filtro
    frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # converte o frame pra HSV
    frame_threshold = cv2.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V)) # tira tudo que não tá no filtro da imagem
    cv2.imshow('padrao', frame) # mostra o frame antes do filtro
    cv2.imshow('filtrado', frame_threshold) # mostra o pós-filtro
    
    return frame_threshold
    
def distance_to_object(known_width, focal_length, percieved_width):
    return (known_width * focal_length) / percieved_width # esse cálculo é explicado no artigo
    
def find_object(frame):
    height, width = frame.shape
    contours, img = cv2.findContours(frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) # encotra os contornos 
    for contour in contours:
        contourArea = cv2.contourArea(contour)
        x,y,w,h = cv2.boundingRect(contour) # pega um retangulo baseado no contorno
        ratio = w/h # proporção entre os lados
        density = contourArea/(w*h) # densidade, ou seja, a minha imagem tá esburacada?
        if evaluate(ratio, density): # se ele parecer com o retângulo do jogo de 2020 e não estiver esburacado...
            rectangle = [x, y, w, h]
            drawn_frame = frame
            cv2.rectangle(frame, (x,y), (x+w, y+h), (255,255,255), 2) # desenha o retângulo da fita na imagem
            cv2.imshow('frame', frame) # mostra a imagem com o retângulo desenhado
            return rectangle

def evaluate(ratio, density):
    if ratio > 0.5 and ratio < 1: # valores por tentativa e erro
        if density > 0.5 and density < 0.9: # valores por tentativa e erro
            return True
    return False

def main():
    # espera para que o RPi esteja sempre ligado
    time.sleep(2.0)
    # configuração da câmera
    cap = cv2.VideoCapture(0)
    cap.set(3,640) # altera width
    cap.set(4,480) # altera height
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 105) # diminui o brilho da câmera pra ficar mais fácil de ver a fita

    # configuração da NetworkTables
    ntinst = NetworkTablesInstance.getDefault()

    print("Setting up NetworkTables client for team {}".format(team))
    ntinst.startClientTeam(team)

    shuffle = ntinst.getTable('Shuffleboard')
    sd = shuffle.getSubTable('Vision') # na minha tabela de visão da shuffleboard, eu mando os meus dados


    # loop infinito para análise da imagem
    while(True):
         
        ret, frame = cap.read() # lê a imagem da câmera
        
        proc = process(frame) # processa a imagem
                
        obj = find_object(proc) # procura o objeto
        
        if obj:
            '''
                Comandos usados para calibrar a largura focal da câmera.
                focal_length = (obj[3] * KNOWN_DISTANCE) / KNOWN_WIDTH
                print("Distância focal: {0}".format(focal_length))
            '''
            center = obj[0] + (obj[2] / 2) # centro da imagem
            diff = (640/2) - center + STANDART_ERROR # ângulo do robô pra fita
            dist = distance_to_object(KNOWN_WIDTH, F, obj[3]) # distância do robô pra fita
            print("Dist: {0} Diff: {1}".format(dist, diff))
            sd.putNumber('Difference', diff) # ângulo
            sd.putNumber('Distance', dist)

        if cv2.waitKey(1) == 27:
            break
                
    cap.release()

if __name__ == "__main__":
    main()
