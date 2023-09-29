import socket

# Define the Raspberry Pi's IP address and port
server_ip = '192.168.12.12'  # Replace with your Raspberry Pi's IP address
server_port = 8888  # Use the same port as in the Raspberry Pi script
# Create a socket object
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Connect to the server
client_socket.connect((server_ip, server_port))
# Specify where you want to save the received image
received_image_path = 'received_image.jpeg'
# Receive image data from the Raspberry Pi and save it to a file
with open(received_image_path, 'wb') as image_file:
    while True:        
        data = client_socket.recv(1024)
        if not data:            
            break
        image_file.write(data)
print(f"Received and saved the image as '{received_image_path}'.")
# Close the client socket
client_socket.close()