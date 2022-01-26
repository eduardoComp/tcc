import Adafruit_BBIO.UART as UART
import serial


def send_command(posT, direction):
     
     #return 1
     
     posT = str(posT) + ";" + direction
     print(posT)
     posT = posT.encode()
     
     UART.setup("UART1")

     ser = serial.Serial(port = "/dev/ttyO1", baudrate=9600, timeout=3)
     ser.close()
     ser.open()
     
     if ser.isOpen():
          print("Serial is open!")
          ser.write(posT)
          r = ser.readline().decode()
          print(r)
          if r == "ok":
               ser.close()
               return 1
          else:
               for i in range(0, 5):
                    ser.write(posT)
                    r = ser.readline().decode()
                    if r == "ok":
                         ser.close()
                         return 1
               ser.close()
               return 0
# send_command(90, "+roll")