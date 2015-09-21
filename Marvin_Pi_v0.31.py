import wiringpi2
import time
import subprocess
from threading import *


from subprocess import call
#THIS IS THE PRIORITY IT WILL RUN AT
#wiringpi2.piHiPri(99)


fileLock = Lock()
arduinoOutLock = Lock()
arduinoInReady = Lock()
arduinoListenerReady = Event()
fileLocation = ""
arduinoIn = -1
arduinoOut = -1
flightPath = []

def fileOpen():
  global fileLocation
  fileLocation = "/home/pi/python/logs/" \
  + str(time.strftime('%c')) + "myLog.txt"
  fileLock.acquire()
  file = open(fileLocation,"w")  
  file.write("Flight Log (")
  file.write(time.strftime("%c"))
  file.write(")\n")
  file.close()
  fileLock.release()

def fileAppend(loc, message):
  fileLock.acquire()
  file = open(loc, "a")
  file.write(message)
  file.close()
  fileLock.release()

def arduinoListen():
  global wiringpi2
  global arduinoIn
  global arduino

  wiringpi2.wiringPiSetup()
  arduino = wiringpi2.serialOpen("/dev/ttyACM0", 9600)
  wiringpi2.delay(250)
  wiringpi2.pinMode(0,1)
  wiringpi2.digitalWrite(0, 1)

  arduinoInReady.acquire()
  arduinoListenerReady.set()
  while(True):
    while(True):
      if(wiringpi2.serialDataAvail(arduino) > 0):
        arduinoIn = chr(wiringpi2.serialGetchar(arduino))
        arduinoInReady.release()
        time.sleep(0.5)
        break
      else:
        time.sleep(0.5)
    arduinoInReady.acquire()

def outputArduino(mes):
  arduinoOutLock.acquire()
  wiringpi2.serialPutchar(arduino, mes)
  arduinoOutLock.release()

def handleFlight():
  arduinoListenerReady.wait()
  launch()
  global arduinoIn

  for command in flightPath:
    command = command.split(":")
    mode = command[0]
    end = command[1]

    if(mode == "HOVER_STEADY"):
      arduino_Out = Thread(target = outputArduino, args = (72,)) #'H'
      arduino_Out.start()
    elif(mode == "LAND"):
      arduino_Out = Thread(target = outputArduino, args = (76,)) #'L'
      arduino_Out.start()      
      file_Out = Thread(target = fileAppend,
        args = (fileLocation,"Landing Sequence begun at " \
        + str(time.time()) + "\n"))
      file_Out.start()
    elif(mode == "END"):
      file_Out = Thread(target = fileAppend,
        args = (fileLocation,"Shutdown Proceeding at " \
        + str(time.time()) + "\n"))
      file_Out.start()
      time.sleep(2)
      subprocess.call(["sudo","shutdown","-h","now"])

    if(end == "TIMER"):
      time.sleep(eval(command[2]))
      next
    else:
      while(True):
        arduinoInReady.acquire()
        if(arduinoIn == end):
          arduinoIn = -1
          arduinoInReady.release()
          break
        else:
          arduinoIn = -1
          arduinoInReady.release()


def launch():
  global arduinoIn

  while(True):
    arduinoInReady.acquire()
    if(arduinoIn == '!'):
      arduino_Out = Thread(target = outputArduino, args = (82,)) #'R'
      arduino_Out.start()
      arduinoIn = -1
      file_Out = Thread(target = fileAppend,
        args = (fileLocation,"Handshake received at " \
        + str(time.time()) + "\n"))
      file_Out.start()
      arduinoInReady.release()
      break
    else:
      arduinoIn = -1
      arduinoInReady.release()

  while(True):
    arduinoInReady.acquire()
    if(arduinoIn == 'r'):
      arduino_Out = Thread(target = outputArduino, args = (84,)) #'T'
      arduino_Out.start()
      arduinoIn = -1
      file_Out = Thread(target = fileAppend,
        args = (fileLocation,"Takeoff Sequence begun at " \
        + str(time.time()) + "\n"))
      file_Out.start()
      arduinoInReady.release()
      break
    else:
      arduinoIn = -1
      arduinoInReady.release()

  while(True):
    arduinoInReady.acquire()
    if(arduinoIn == 't'):
      arduino_Out = Thread(target = outputArduino, args = (72,)) #'H'
      arduino_Out.start()
      arduinoIn = -1
      file_Out = Thread(target = fileAppend,
        args = (fileLocation,"Takeoff Sequence completed at " \
        + str(time.time()) + "\n"))
      file_Out.start()
      arduinoInReady.release()
      break
    else:
      arduinoIn = -1
      arduinoInReady.release()

def main():
  #Setup
  fileCreator = Thread(target = fileOpen)
  fileCreator.start()
  arduinoListenerReady.clear()

  global flightPath
  #change as desired
  flightPath = ["HOVER_STEADY:TIMER:20", "LAND:l", "END:TIMER:5"]

  inputHandler = Thread(target = handleFlight)
  inputHandler.start()

  arduinoListener = Thread(target = arduinoListen)
  arduinoListener.start()

#----------------------------------------------------------------------------------------------------------------------------

main()
