import cv2, math, time, numpy, imutils, threading
from networktables import NetworkTablesInstance

    
KNOWN_DISTANCE = 118 # distância real do objeto para a câmera (só deve ser usada para calibrar a largura focal da câmera)
KNOWN_WIDTH = 55 # largura real do objeto (usada para calcular a distância da câmera para o objeto)
F = 10 # largura focal (calculada por meio de testes)
STANDART_ERROR = 105

team = 6404
dist = 0 # variável para armazenar a distância da câmera para o objeto

def process(frame): # método pra processar as imagens
    
    # Menores valores possiveis pra Threshold HSV (peguei do GRIP)
    low_H = 50
    low_S = 50
    low_V = 150

    # Maiores valores possiveis para Threshold HSV (peguei do GRIP)
    high_H = 100
    high_S = 200
    high_V = 255
    
    # filtro
    frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # converte o frame pra HSV
    frame_threshold = cv2.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V)) # troca os frames que nao batem com os valores pra preto
    cv2.imshow('padrao', frame)
    cv2.imshow('filtrado', frame_threshold)
    
    return frame_threshold
    
def distance_to_object(known_width, focal_length, percieved_width):
    return (known_width * focal_length) / percieved_width
    
def find_object(frame, bbox=[0, 0, 640, 480]):
    x1, y1, w, h = bbox
    x2 = x1 + w
    y2 = y1 + h
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
    roi = frame[y1:y2, x1:x2]
    height, width = frame.shape
    contours, img = cv2.findContours(roi, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    print(len(contours))
    for contour in contours:
        contourArea = cv2.contourArea(contour)
        x,y,w,h = cv2.boundingRect(contour) # pega um retangulo baseado no contorno
        ratio = w/h
        print([x, y, w, h])
        density = contourArea/(w*h)
        if evaluate(ratio, density):
            rectangle = [x, y, w, h]
            drawn_frame = frame
            cv2.rectangle(roi, (x,y), (x+w, y+h), (255,255,255), 2)
            cv2.imshow('frame', frame)
            return rectangle

def evaluate(ratio, density):
    if ratio > 0.5 and ratio < 1:
        if density > 0.5 and density < 0.9:
            return True
    return False

def main():
    # espera para que o RPi esteja sempre ligado
    time.sleep(2.0)
    # configuração da câmera
    cap = cv2.VideoCapture(0)
    cap.set(3,640) # altera width
    cap.set(4,480) # altera height
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 105)

    # configuração da NetworkTables
    ntinst = NetworkTablesInstance.getDefault()

    print("Setting up NetworkTables client for team {}".format(team))
    ntinst.startClientTeam(team)

    shuffle = ntinst.getTable('Shuffleboard')
    sd = shuffle.getSubTable('Vision')


    # loop infinito para análise da imagem
    while(True):
         
        ret, frame = cap.read()
        
        proc = process(frame)
                
        obj = find_object(proc)
        
        if obj:
            '''
                Comandos usados para calibrar a largura focal da câmera.
                focal_length = (obj[3] * KNOWN_DISTANCE) / KNOWN_WIDTH
                print("Distância focal: {0}".format(focal_length))
            '''
            # print(obj)
            center = obj[0] + (obj[2] / 2)
            diff = (640/2) - center + STANDART_ERROR
            dist = distance_to_object(KNOWN_WIDTH, F, obj[3])
            # print("Dist: {0} Diff: {1}".format(dist, diff))
            sd.putNumber('Difference', diff)
            sd.putNumber('Distance', dist)

        if cv2.waitKey(1) == 27:
            break
                
    cap.release()

if __name__ == "__main__":
    main()
