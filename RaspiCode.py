import requests
import base64
import random
from PIL import Image
from io import BytesIO
import os
import time
import base64
from io import BytesIO

# Function to generate random coordinates
def generate_random_coordinates():
    latitude = random.uniform(-90, 90)
    longitude = random.uniform(-180, 180)
    return latitude, longitude

# Function to generate dummy NPK and moisture values (for now)
def generate_dummy_values():
    nitrogen = 23
    phosphorus = 23
    potassium = 23
    return nitrogen, phosphorus, potassium


# Function to capture image and encode it to base64
def capture_and_encode_image():
    try:
        IMAGE_PATH = "assets/images/capture.png"
        os.system("libcamera-still --width 1024 --height 1024 -o {}".format(IMAGE_PATH))
        image = Image.open(r"{}".format(IMAGE_PATH))

        image_io = BytesIO()
        image.save(image_io, format='PNG')
        image_bytes = image_io.getvalue()
        image64 = base64.b64encode(image_bytes).decode('ascii')

    except Exception as e:
        print(e)
        return False
    
    return image64

# Function to send data to API
def send_data_to_api(coordinates, nitrogen, phosphorus, potassium, image_base64):
    url = "https://grubworm-full-dory.ngrok-free.app/api/analysis/remoteStore"
    data = {
        "latitude": coordinates[0],
        "longitude": coordinates[1],
        "nitrogen":nitrogen, 
        "phosphorus":phosphorus, 
        "potassium":potassium,
        "image": image_base64
    }
    response = requests.post(url, json=data)
    if response.status_code == 200:
        print("Data sent successfully.")
        print(response.text)
    else:
        print("Failed to send data. Status code:", response.status_code)

# Main function
def main():
    # Generate random coordinates
    coordinates = generate_random_coordinates()

    # Generate dummy NPK and moisture values
    nitrogen, phosphorus, potassium = generate_dummy_values()

    # Capture and encode image
    image_base64 = capture_and_encode_image()

    # Send data to API
    send_data_to_api(coordinates, nitrogen, phosphorus, potassium, image_base64)

if __name__ == "__main__":
    main()
    # capture_and_encode_image()