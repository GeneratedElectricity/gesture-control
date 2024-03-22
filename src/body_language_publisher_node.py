#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import mediapipe as mp
import cv2
import math

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

class HandAnglePublisher(Node):
    def __init__(self):
        super().__init__('hand_angle_publisher')
        self.publisher_ = self.create_publisher(Float64, 'hand_angle', 10)
        self.timer_ = self.create_timer(0.05, self.publish_hand_angle)

        self.cap = cv2.VideoCapture(0)

    def publish_hand_angle(self):
        try:
            ret, frame = self.cap.read()
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = cv2.flip(image, 1)
            image.flags.writeable = False

            with mp_hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.5, max_num_hands=1) as hands:
                results = hands.process(image)

            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            if results.multi_hand_landmarks and len(results.multi_hand_landmarks) == 1:
                hand = results.multi_hand_landmarks[0]

                image_height, image_width, _ = image.shape
                point_12 = (int(hand.landmark[12].x * image_width), int(hand.landmark[12].y * image_height))
                point_0 = (int(hand.landmark[0].x * image_width), int(hand.landmark[0].y * image_height))
                vertical_line_start = (point_0[0], 0)  
                vertical_line_end = (point_0[0], image_height) 
                cv2.line(image, (int(vertical_line_start[0]), int(vertical_line_start[1])), 
                         (int(vertical_line_end[0]), int(vertical_line_end[1])), (139, 0, 0), 2)

                line_0_to_12_start = (int(point_0[0]), int(point_0[1]))
                line_0_to_12_end = (int(point_12[0]), int(point_12[1]))

                cv2.line(image, line_0_to_12_start, line_0_to_12_end, (0, 0, 255), 2)

                angle_rad = math.atan2(line_0_to_12_end[1] - line_0_to_12_start[1],
                                       line_0_to_12_end[0] - line_0_to_12_start[0]) - \
                            math.atan2(vertical_line_end[1] - vertical_line_start[1],
                                       vertical_line_end[0] - vertical_line_start[0])

                angle_deg = math.degrees(angle_rad)

                if angle_deg < 0:
                    angle_deg += 180

                cv2.putText(image, f'Angle: {angle_deg:.2f} degrees', (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

                # Publish the angle
                msg = Float64()
                msg.data = angle_deg
                self.publisher_.publish(msg)

            cv2.imshow('Hand Tracking', image)

            if cv2.waitKey(10) & 0xFF == ord('q'):
                self.destroy_node()
                cv2.destroyAllWindows()
                self.cap.release()
                rclpy.shutdown()

        except KeyboardInterrupt:
            self.destroy_node()
            cv2.destroyAllWindows()
            self.cap.release()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    hand_angle_publisher = HandAnglePublisher()
    rclpy.spin(hand_angle_publisher)
    hand_angle_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
