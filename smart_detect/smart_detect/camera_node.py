import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from smart_detect.msg import Objects  # custom message

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.sub = self.create_subscription(Objects, '/obj/data', self.obj_callback, 10)
        self.cap = cv2.VideoCapture(0)
        self.objects = []

        self.timer = self.create_timer(0.05, self.timer_callback)  # ~20 FPS

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Publish raw frame
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.pub.publish(msg)

            # Draw bounding boxes if objects detected
            for obj in self.objects:
                color = (0,255,0)  # default green
                if obj.distance < 5:
                    color = (0,0,255)  # red
                elif obj.distance <= 10:
                    color = (0,255,255)  # yellow

                cv2.rectangle(frame, (obj.xmin, obj.ymin), (obj.xmax, obj.ymax), color, 2)
                cv2.putText(frame, f"{obj.label}:{obj.distance:.1f}m", 
                            (obj.xmin, obj.ymin-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            cv2.imshow("Camera Feed", frame)
            cv2.waitKey(1)

    def obj_callback(self, msg):
        self.objects = msg.objects

def main():
    rclpy.init()
    node = CameraNode()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
