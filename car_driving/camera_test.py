from picamera import PiCamera
from time import sleep


print("start camera...")
camera = PiCamera()
camera.resolution = (640,480)
camera.start_preview()
sleep(2)
print("capture a photo, cheese...")
camera.capture("/home/pi/temp/cap3.jpg")
#print("recording a video...")
#camera.start_recording("/home/pi/temp/vid1.h264")
#camera.wait_recording(10)
#camera.stop_recording()
#sleep(5)
#camera.stop_preview()
print("done.")



