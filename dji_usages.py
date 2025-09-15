from djitellopy import Tello
import cv2

tello = Tello()

# 連線、起飛、落地、電池
tello.connect()
tello.takeoff()
tello.land()
tello.get_batttery()

# send int
tello.move_left() 
tello.move_right()
tello.move_up()

# send int, int = degree
tello.rotate_clockwise()
tello.rotate_counter_clockwise()


"""
move(direction: str, distance: int)
direction: up, down, left, right, forward, back
distance: 20 ~ 500 （單位：公分）
"""
tello.move_back()
tello.move_forward()
tello.move_left()
tello.move_right()


"""
拍照用法
"""
tello.streamon()
tello.streamoff()
frame_read= tello.get_frame_read()
cv2.imwrite("pic.png", frame_read.frame)

