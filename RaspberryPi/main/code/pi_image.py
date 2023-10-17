import socket
import json
from picamera import PiCamera

camera = PiCamera()
camera.capture('/home/pi/Desktop/A2/to_send/image.jpg')

# Function to send a JPEG file to the client
def send_jpeg(client_socket,path):
    print("Sending JPEG image...")
    with open(path, 'rb') as file:
        data = file.read(1024)
        while data:
            client_socket.send(data)
            data = file.read(1024)
    print("JPEG image sent.")
        

# Function to receive a JSON file from the client
def receive_json(client_socket):
    print("Receiving JSON data...")
    json_data = b''
    while True:
        data = client_socket.recv(1024)
        if not data:
            break
        json_data += data
    print("JSON data received:")
    return json_data.decode('utf-8')

def connect_socket():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', 12345))
    server_socket.listen(1)
    print("Waiting for a connection...")
    client_socket, client_address = server_socket.accept()
    return client_socket, client_addres



#preceived_json = receive_json(client_socket)


# Print the received JSON data
#print(received_json)

#client_socket.close()
#server_socket.close()
