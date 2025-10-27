import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class TelloController(Node):
    def __init__(self):
        super().__init__('tello_controller')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/tello/camera/image_raw', self.image_callback, 10)

        # --- OpenCV HSV trackbars for color thresholding ---
        cv2.namedWindow('controls', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('controls', 520, 180)
        cv2.createTrackbar('low H', 'controls', 0, 179, lambda v: None)
        cv2.createTrackbar('high H', 'controls', 179, 179, lambda v: None)
        cv2.createTrackbar('low S', 'controls', 0, 255, lambda v: None)
        cv2.createTrackbar('high S', 'controls', 255, 255, lambda v: None)
        cv2.createTrackbar('low V', 'controls', 0, 255, lambda v: None)
        cv2.createTrackbar('high V', 'controls', 255, 255, lambda v: None)

    def process_and_draw(self, frame_bgr):
        """
        Read HSV range from trackbars, threshold the image,
        find the largest contour, and draw a bounding box.
        Shows three windows: original, mask, and result overlay.
        """
        # Read trackbar values
        H_low  = cv2.getTrackbarPos('low H',  'controls')
        H_high = cv2.getTrackbarPos('high H', 'controls')
        S_low  = cv2.getTrackbarPos('low S',  'controls')
        S_high = cv2.getTrackbarPos('high S', 'controls')
        V_low  = cv2.getTrackbarPos('low V',  'controls')
        V_high = cv2.getTrackbarPos('high V', 'controls')
        hsv_low  = np.array([H_low,  S_low,  V_low],  dtype=np.uint8)
        hsv_high = np.array([H_high, S_high, V_high], dtype=np.uint8)

        # Convert to HSV and threshold
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsv_low, hsv_high)

        # Optional morphology to clean noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        # Find contours and draw the largest bounding box
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # ---- contour visualization (red) like the slide ----
        # create black image and draw all contours in red for inspection
        show_image = cv2.cvtColor(np.zeros(frame_bgr.shape[:2], dtype=np.uint8), cv2.COLOR_GRAY2BGR)
        if contours:
            cv2.drawContours(show_image, contours, -1, (0, 0, 255), 2)

        overlay = frame_bgr.copy()
        chosen = None
        best_area = 0.0
        if contours:
            for c in contours:
                area = cv2.contourArea(c)
                if area < 500:
                    continue
                # prefer rectangle-like contours (approx 4 vertices)
                peri = cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, 0.02 * peri, True)
                is_rect_like = len(approx) == 4
                # scoring: rectangle-like gets area bonus
                score = area * (1.5 if is_rect_like else 1.0)
                if score > best_area:
                    best_area = score
                    chosen = c
            # fallback: if none passed the area threshold, pick absolute largest
            if chosen is None and contours:
                chosen = max(contours, key=cv2.contourArea)
                best_area = cv2.contourArea(chosen)
            if chosen is not None and best_area > 0:
                # --- Bounding Rect center (Approximate) ---
                x, y, w, h = cv2.boundingRect(chosen)
                br_cx, br_cy = int(x + w * 0.5), int(y + h * 0.5)
                cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(overlay, (br_cx, br_cy), 4, (0, 255, 0), -1)
                cv2.putText(overlay, "BR", (br_cx + 6, br_cy - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                cv2.putText(overlay, f"area:{int(best_area)}", (x, max(0, y - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                # --- Moments center ---
                M = cv2.moments(chosen)
                if M["m00"] != 0:
                    mo_cx = int(M["m10"] / M["m00"])
                    mo_cy = int(M["m01"] / M["m00"])
                    cv2.circle(overlay, (mo_cx, mo_cy), 4, (255, 0, 0), -1)
                    cv2.putText(overlay, "M", (mo_cx + 6, mo_cy - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
                    print("Moments center:", mo_cx, mo_cy)

                # --- Mean center (average of contour points) ---
                mean_xy = chosen.reshape(-1, 2).mean(axis=0)
                mn_cx, mn_cy = int(mean_xy[0]), int(mean_xy[1])
                cv2.circle(overlay, (mn_cx, mn_cy), 4, (0, 255, 255), -1)
                cv2.putText(overlay, "Mean", (mn_cx + 6, mn_cy - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
                print("Average:", mn_cx, mn_cy)

                # For reference also print the BR center
                print("Approximate (BR):", br_cx, br_cy)

        # Show images
        cv2.imshow('frame', frame_bgr)
        cv2.imshow('mask', mask)
        cv2.imshow('res', overlay)
        cv2.imshow('contours', show_image)
        cv2.waitKey(1)

    def img_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            # convert to BGR for consistent processing in OpenCV drawing
            bgr = cv2.cvtColor(self.img, cv2.COLOR_RGB2BGR)
            self.process_and_draw(bgr)
        except Exception as e:
            print(e)

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return
        try:
            self.process_and_draw(cv_image)
        except Exception as e:
            pass

    def control_timer_callback(self):
        pass  # existing method content unchanged

def main(args=None):
    rclpy.init(args=args)
    node = TelloController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()