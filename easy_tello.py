import socket
import threading
import time

tello1_address = ('192.168.10.1', 8889)
local1_address = ('192.168.10.2', 9000)

sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock1.bind(local1_address)

def send(message, delay):
  try:
    sock1.sendto(message.encode(), tello1_address)
    print("Sending message: " + message)
  except Exception as e:
    print("Error sending: " + str(e))

  time.sleep(delay)


def receive():
  while True:
    try:
      response1, ip_address = sock1.recvfrom(128)
      print("Received message: from Tello : " + response1.decode(encoding='utf-8'))

    except Exception as e:
      sock1.close()
      
      print("Error receiving: " + str(e))
      break


receiveThread = threading.Thread(target=receive)
receiveThread.daemon = True
receiveThread.start()

#Tello uses cm units by default.
distance = 60
yaw_angle = 90

# Yaw clockwise (right)
yaw_direction = "cw"

# Put Tello into command mode
send("command", 3)
send("takeoff", 8)

# Loop and create each leg of the box
for i in range(1):
  send("forward " + str(distance), 5)
  send('battery?' , 1)
  send("back " + str(distance), 5)
  send("cw " + str(yaw_angle), 3)


send("land", 5)

print("Mission completed successfully!")
sock1.close()
