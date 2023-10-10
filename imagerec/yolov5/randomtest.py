import requests

url = "http://127.0.0.1:5000/image"
file_path = "1665419523_2_C.jpg"

with open(file_path, "rb") as file:
    files = {"file": file}
    response = requests.post(url, files=files)

print(response.text) #Print the response content as text
#print(response.json())