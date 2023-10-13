from picamera import PiCamera

camera = PiCamera()
camera.capture("test_img.jpg")
camera.close()