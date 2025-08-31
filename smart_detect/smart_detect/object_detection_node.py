import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from smart_detect.msg import Object, Objects   # <- custom ROS 2 messages
from ultralytics import YOLO
import cv2

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.bridge = CvBridge()

        # Subscribe to camera feed
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.callback, 10
        )
        # Publisher for detected objects
        self.pub = self.create_publisher(Objects, '/obj/data', 10)

        # Load YOLOv8 model (nano version for speed)
        self.model = YOLO("yolov8n.pt")
        self.get_logger().info("YOLOv8 model loaded successfully.")

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Run YOLOv8 inference
        results = self.model.predict(frame, conf=0.5, verbose=False)

        objs_msg = Objects()

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls)
                label = self.model.names[cls_id]
                conf = float(box.conf)

                # Bounding box
                xyxy = box.xyxy.cpu().numpy()[0]
                xmin, ymin, xmax, ymax = map(int, xyxy)

                # ---- Approx distance calculation ----
                bbox_width = xmax - xmin
                # crude mapping: larger bbox = closer object
                distance = max(1.0, min(20.0, 500 / bbox_width))

                obj = Object()
                obj.label = label
                obj.distance = round(distance, 1)
                obj.xmin, obj.ymin, obj.xmax, obj.ymax = xmin, ymin, xmax, ymax
                objs_msg.objects.append(obj)

        # Publish list of objects
        self.pub.publish(objs_msg)

def main():
    rclpy.init()
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
