import cv2
from djitellopy import Tello
import time

tello = Tello()
tello.connect(False)

tello.streamon()
frame_read = tello.get_frame_read()

tello.takeoff()
# tello.move_up(60)
# tello.flip_back()
# tello.rotate_counter_clockwise(180)
# tello.rotate_clockwise(180)
# cv2.imwrite("picture22.png", frame_read.frame)


for i in range(4):
    # tello.move_forward(20)

    if i % 2 == 0:
        tello.flip_forward()
        print(f"在第 {i+1} 個角，向前翻轉")
    else:
        tello.flip_back()
        print(f"在第 {i+1} 個角，向後翻轉")
    time.sleep(1)

cv2.imwrite("picture.png", frame_read.frame)

tello.land()