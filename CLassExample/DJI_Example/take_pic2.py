import cv2
from djitellopy import Tello

move_length = 30

tello = Tello()
tello.connect()

tello.streamon()
frame_read = tello.get_frame_read()

tello.takeoff()

tello.move_forward(move_length)
tello.rotate_counter_clockwise(90)
tello.move_forward(move_length)
tello.rotate_counter_clockwise(90)
tello.move_forward(move_length)
tello.rotate_counter_clockwise(90)
tello.move_forward(move_length)
tello.rotate_counter_clockwise(90)

tello.flip_forward()
cv2.imwrite("pic.png", frame_read.frame)


# i = 1 
# for i in range(4):
#     # tello.move_forward(20)

#     if i % 2 == 0:
#         tello.flip_forward()
#         print(f"在第 {i+1} 個角，向前翻轉")
#     else:
#         tello.flip_back()
#         print(f"在第 {i+1} 個角，向後翻轉")
#     i+=1
#     time.sleep(1)

# cv2.imwrite("picture.png", frame_read.frame)

tello.land()