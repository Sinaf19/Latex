from flask import Flask, request, redirect, url_for, render_template, json, render_template_string, jsonify
from flask import Response
from smbus import SMBus
from time import sleep
import os
import threading
import argparse
import datetime
import imutils
import time
import cv2
from imutils.video import VideoStream
from pyzbar import pyzbar
import datetime
import time

app = Flask(__name__, template_folder='template', static_folder='static')

color = '#C9C3AB'
addr = 0x8
bus =SMBus(1)

OnOff_Avant = 0
OnOff_Arriere = 0
OnOff_Gauche = 0
OnOff_Droite = 0
scaleEvent = 0
PWM = 100
PWMsending = 100

# initialize the output frame and a lock used to ensure thread-safe
# exchanges of the output frames (useful when multiple browsers/tabs
# are viewing the stream)
outputFrame = None
lock = threading.Lock()

# initialize the video stream and allow the camera sensor to
# warmup
vs = VideoStream(usePiCamera=1).start()
time.sleep(2.0)


# CODE ENVOI:
#0 = stop
#1 = En avant
#2 = En arrière
#3 = 5m en avant
#4 = tourner sur soi
#5 = LED coloré
#6 = moteur droite
#7 = moteur gauche 
#8 = ordre musique
#9 = ordre sequence led
#10 = toujours tout droit???? pq?


# Renvoie aux pages HTML
@app.route("/")
def startPage():
    return render_template('ProjetRobotAutonome.html')


@app.route("/ProgrammesPrincipaux")
def ProgrammesPrincipaux():
    return render_template('ProgrammesPrincipaux.html')


@app.route("/VisionCamera")
def VisionCamera():
    return render_template('VisionCamera.html')


@app.route("/ControleRobot")
def ControleRobot():
    return render_template('ControleRobot.html')

# Fonction de récupération du slider


@app.route("/slider", methods=['GET', 'POST'])
def route():
    global PWMsending

    if request.method == 'POST':
        if request.form.get('slide'):
            PWM = request.form.get('slide')
            PWMsending = int(PWM)
            print('PWM : ', PWM)
            return json.dumps({'PWM': PWM})
    return("nothing")

 
 # Fonctions du contrôle du robot: avant, arrière, gauche, droite


@app.route("/avant")
def avant():
    global OnOff_Arriere
    global OnOff_Avant
    global OnOff_Droite
    global OnOff_Gauche

    if OnOff_Avant % 2 == 0:
        print("avant On")
        send_order(1)
    if OnOff_Avant % 2 != 0:
        print("avant stop")
        send_order(0)

    if OnOff_Droite > 0 or OnOff_Arriere > 0 or OnOff_Gauche > 0:
        OnOff_Arriere = 0
        OnOff_Gauche = 0
        OnOff_Droite = 0

    OnOff_Avant += 1
    return("nothing")


@app.route("/arriere")
def arriere():
    global OnOff_Arriere
    global OnOff_Avant
    global OnOff_Gauche
    global OnOff_Droite

    if OnOff_Arriere % 2 == 0:
        print("arriere on")
        send_order(2)

    if OnOff_Arriere % 2 != 0:
        print("arriere stop")

        send_order(0)

    if OnOff_Droite > 0 or OnOff_Avant > 0 or OnOff_Gauche > 0:
        OnOff_Avant = 0
        OnOff_Gauche = 0
        OnOff_Droite = 0

    OnOff_Arriere += 1
    print(OnOff_Arriere)
    return("nothing")


@app.route("/gauche")
def gauche():
    global OnOff_Arriere
    global OnOff_Avant
    global OnOff_Gauche
    global OnOff_Droite

    if OnOff_Gauche % 2 == 0:
        print("gauche On")
        send_order(7)

    if OnOff_Gauche % 2 != 0:
        print("gauche Stop")
        send_order(0)

    if OnOff_Droite > 0 or OnOff_Avant > 0 or OnOff_Arriere > 0:
        OnOff_Avant = 0
        OnOff_Arriere = 0
        OnOff_Droite = 0

    OnOff_Gauche += 1
    print(OnOff_Gauche)
    return("nothing")


@app.route("/droite")
def droite():
    global OnOff_Arriere
    global OnOff_Avant
    global OnOff_Gauche
    global OnOff_Droite

    if OnOff_Droite % 2 == 0:
        send_order(6)
        print("droite On")
    if OnOff_Droite % 2 != 0:
        send_order(0)
        print("droite Stop")

    if OnOff_Arriere > 0 or OnOff_Avant > 0 or OnOff_Gauche > 0:
        OnOff_Avant = 0
        OnOff_Gauche = 0
        OnOff_Arriere = 0

    OnOff_Droite += 1
    print(OnOff_Droite)
    return("nothing")

#Envoie de différents programme

@app.route("/mode5m")
def mode5m():
	send_order(3)#A DEFINIR
	return("nothing")

@app.route("/tournersursois")
def tournersursois():
	send_order(4)#A DEFINIR
	return("nothing")


#Autre commande 
@app.route("/retour")
def retour():
	send_order(5)
	send_order(0)
	return("nothing")
@app.route("/envoierouge")
def envoierouge():
	send_order(0)
	return("nothing")


  # Envoie de l'information à l'arduino


def send_order(command):
    bus.write_byte(addr,command)
    print(command)





def streaming():
    # grab global references to the video stream, output frame, and
    # lock variables
    global vs, outputFrame, lock


    # loop over frames from the video stream
    while True:
        # read the next frame from the video stream, resize it,
 
        frame = vs.read()
        frame = imutils.resize(frame, width=400)

        # acquire the lock, set the output frame, and release the
        # lock
        with lock:
            outputFrame = frame.copy()


def generate():
    # grab global references to the output frame and lock variables
    global outputFrame, lock

    # loop over frames from the output stream
    while True:
        # wait until the lock is acquired
        with lock:
            # check if the output frame is available, otherwise skip
            # the iteration of the loop
            if outputFrame is None:
                continue

                # encode the frame in JPEG format
            (flag, encodedImage) = cv2.imencode(".jpg", outputFrame)

            # ensure the frame was successfully encoded
            if not flag:
                continue

            # yield the output frame in the byte format
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
              bytearray(encodedImage) + b'\r\n')


@app.route("/video_feed")
def video_feed():
    # return the response generated along with the specific media
    # type (mime type)
    return Response(generate(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/suivreligne")
def suivreligne():
    #grab global variable to use
    global outputFrame, vs, lock

    while(True):
        ret = vs.read()
        crop_img = vs.read()
        crop_img = imutils.resize(crop_img, height = 60, width = 160)

        #convert to grayscale
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        
        #gaussian blur
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        #color threshold
        ret, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)
        
        #find the contours of the frame
        contours, hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)
        
        #find the biggest contour (if detected)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            
            #draw the contours and lines onto the initial cropped image
            cv2.line(crop_img, (cx, 0), (cx, 720), (255, 0, 0), 1)
            cv2.line(crop_img, (0, cy), (1280, cy), (255, 0, 0), 1)
            
            cv2.drawContours(crop_img, contours, -1, (0, 255, 0), 1)
            
            #the robot detectsthe positioning of the line and figures out which way to turn
            if cx >= 120:
                print ("Turn Left!")
                #bus.write_byte(addr, 7)

                
            if cx < 120 and cx >50:
                print ("On Track!")
                #bus.write_byte(addr, 1)
                
            if cx <= 50:
                print ("Turn Right!")
                #bus.write_byte(addr, 6)
                    
            else: 
                print ("I don't see the line")
        cv2.imshow('frame', crop_img)
    return("nothing")


@app.route("/qrcode")
def qrcode():

                #grab global values
    global outputFrame, vs, lock

    while(True):
        #grab the frame frome the output video and resize it
        qr_frame = vs.read()
        qr_frame = imutils.resize(qr_frame, width=400)
        #find the barcodes in the frame and decode each of the barcodes
        barcodes = pyzbar.decode(qr_frame)

        #loop over the detected barcodes
        for barcode in barcodes:
            #extract the bounding box location of the barcode and draw the bounding box surrounding the barcode on the image
            (x, y, w, h) = barcode.rect
            cv2.rectangle(qr_frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

            #the barcode data is a bytes object so if we want to draw it on our output image we need to convert it to a string first
            barcodeData = barcode.data.decode("utf-8")
            barcodeType = barcode.type

            #draw the barcode data and barcode type on the image
            text = "{} ({})".format(barcodeData, barcodeType)
            cv2.putText(qr_frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)



            if barcodeData == "RIGHT":
                print("Go right!")
            if barcodeData == "LEFT":
                print("Go left!")
            if barcodeData == "SOUND":
                print("Make some noiiiise")
            if barcodeData == "LIGHT":
                print("sequence rouge bleu vert blanc")
                bus.write_byte(addr, 9)
            if barcodeData == "5MENAVANT":
                print("Tout droit mais sur 5m")
                bus.write_byte(addr, 3)
            if barcodeData == "TOURNESURSOI":
                print("Tourner tourner tourner!")
                bus.write_byte(addr, 4)
            if barcodeData == "TOUTDROIT":
                print("Tout droit pour l'éternité")
                bus.write_byte(addr, 10)
            if barcodeData == "MUSIQUE":
                print("jouer MI DO FA")
                bus.write_byte(addr, 8)
    
            #show the output frame 
            cv2.imshow("Barcode Scanner", qr_frame)
    
    return("nothing")


# Programme principale flask
if __name__ == '__main__':


	
    # start a thread that will perform streaming
    t = threading.Thread(target=streaming)
    t.daemon = True
    t.start()

    # a utiliser si wifi quentin
    # app.run(host="172.20.10.3",port="5000",debug=True,use_reloader=False)
    # a utiliser si mon wifi et ordi
    # app.run(host="192.168.204.1",port="5000",debug=True,use_reloader=False)
    #app.run(host="127.0.0.1", port="5000", debug=True, threaded=True, use_reloader=False)
    # a utiliser si pas hostap
    #app.run(host="172.20.10.12", port="5000", debug=True,
           # threaded=True, use_reloader=False)
    app.run(debug=True)


vs.stop()
