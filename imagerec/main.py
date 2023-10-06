import socket
import json

# Function to send a JSON file to the server
def send_json(server_socket, json_data):
    server_socket.send(json_data.encode('utf-8'))

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.12.12', 12345))

# Receive and save the JPEG image
with open('received_image.jpg', 'wb') as file:
    while True:
        data = client_socket.recv(1024)
        if not data:
            break
        file.write(data)

print("JPEG image received and saved.")

# Create a JSON object to send to the server
# replace json_data with your jsonified data
json_data = {
    "key1": "value1",
    "key2": "value2",
    # Add more data as needed
}

# Send the JSON data to the server
print("Sending JSON data...")
send_json(client_socket, json.dumps(json_data))
print("JSON data sent.")

client_socket.close()