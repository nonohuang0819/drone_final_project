from djitellopy import Tello
import time
import cv2

tello = Tello()

tello.connect()

# tello.move_forward(20)
# tello.rotate_counter_clockwise(90)
# tello.move_forward(20)
# tello.rotate_counter_clockwise(90)
# tello.move_forward(20)
# tello.rotate_counter_clockwise(90)
# tello.move_forward(20)
# tello.flip_forward()
# tello.flip_back()
# tello.move_left(100)
# tello.move_back(50)
# tello.move_right(100)
# tello.rotate_clockwise(90)
# tello.rotate_counter_clockwise(60)
# tello.move_forward(90)

# 開啟影像串流
tello.streamon()
frame_read = tello.get_frame_read()
tello.takeoff()
    
time.sleep(2)

side_length = 100

    # 執行四次，代表四個角
for i in range(4):
    tello.move_forward(side_length)

    if i % 2 == 0:
        tello.flip_forward()
        print(f"在第 {i+1} 個角，向前翻轉")
    else:
        tello.flip_back()
        print(f"在第 {i+1} 個角，向後翻轉")
    time.sleep(1)

    # 3. 拍照
    frame = frame_read.frame
    photo_name = f'corner_{i+1}.jpg'
    cv2.imwrite(photo_name, frame)
    print(f"已拍攝照片：{photo_name}")
        
    time.sleep(1)

    tello.rotate_counter_clockwise(90)


tello.land()
tello.streamoff()