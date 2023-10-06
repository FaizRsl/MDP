import socket
import json

# Function to send a JSON file to the server
def send_json(server_socket, json_data):
    server_socket.send(json_data.encode('utf-8'))

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.12.12', 12345))

# Set a timeout for the socket
client_socket.settimeout(2)
# Receive and save the JPEG image
with open('received_image.jpg', 'wb') as file:
    try:
        while True:
            data = client_socket.recv(1024)
            if not data:
                break
            file.write(data)
    except socket.timeout:
        # Handle socket timeout
        print("Socket timeout occurred. Image transmission may be incomplete.")

# with open('received_image.jpg', 'wb') as file:
#     image_data = b''  # Variable to store the received image data
#     while True:
#         data = client_socket.recv(1024)
#         if not data:
#             break
#         image_data += data

#         # Check if the image transmission is complete
#         if b'\xff\xd9' in image_data:
#             # The marker \xff\xd9 indicates the end of a JPEG image
#             # Write the complete image data to the file
#             file.write(image_data)
#             break

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