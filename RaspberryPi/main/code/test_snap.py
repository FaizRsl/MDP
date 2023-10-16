#!/usr/bin/env python3
import json
import queue
import time
from multiprocessing import Process, Manager
from typing import Optional
import os
import requests
from consts import SYMBOL_MAP
from logger import prepare_logger
from settings import API_IP, API_PORT
from picamera import PiCamera
from time import sleep


def snap_and_rec(obstacle_id_with_signal):
    """
    RPi snaps an image and calls the API for image-rec.
    The response is then forwarded back to the android
    :param obstacle_id_with_signal: the current obstacle ID followed by underscore followed by signal
    """
    obstacle_id, signal = obstacle_id_with_signal.split("_")
    print(f"Capturing image for obstacle id: {obstacle_id}")
    url = f"http://{API_IP}:{API_PORT}/image"
    filename = f"{int(time.time())}_{obstacle_id}_{signal}.jpg"
    command= f"raspistill -o {filename}"
    #camera = PiCamera()
    retry_count = 0
    # extn = ".jpg"
    # rpistr = "libcamera-jpeg -e " + extn + " -n -t 500 -o " + filename

    while retry_count < 6:

        retry_count += 1

        #Capturing image using PiCamera
        os.system(command)
        #camera.start_preview()
        #sleep(3)
        #camera.capture(filename)
        #camera.stop_preview()
        #camera.close()

        #Print the number of times image has been captured
        print(f"Current image capture trial: {retry_count}")

        print("Requesting from image API")
        
        files={"file": open(filename, 'rb')}
        response = requests.post(url, files=files)
        print("requests.post() runs")

        if response.status_code != 200:
            print("Something went wrong when requesting path from image-rec API. Please try again.")
            return

        results = json.loads(response.content)
        
        if results['image_id'] != 'NA':
            break

    print(f"results: {results}")
    print(f"Image recognition results: {results} ({SYMBOL_MAP.get(results['image_id'])})")
    print("snap_and_rec function tested and complete!")
    return

if __name__ == "__main__":
    i = 0
    while i < 10:
        obstacle_id_with_signal = "3_C"
        snap_and_rec(obstacle_id_with_signal)
        i+=1
