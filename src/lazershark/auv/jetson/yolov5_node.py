import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from vision_msgs.msg import Pose2D
from cv_bridge import CvBridge
import numpy as np
import torch, sys, pathlib

class YoloV5Node(Node):
    """
    YOLOv5 object detection node for the Explorer cam.

    Subscribes to /image_raw and publishes coordinates of detections to /detections.

    parameters:
        - yolo_dir: Path to local YOLOv5 repo files
    """

    def __init__(self):
        super().__init__('yolov5_node')
        
        # parameters
        self.declare_parameter('yolo_dir', os.path.expanduser('~/yolov5'))
        self.declare_parameter('weights', 'best.pt')
        self.declare_parameter('conf_thres', 0.25)
        self.declare_parameter('device', 'auto') # auto allows us to switch between CPU and GPU if any failures occur
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('output_topic', '/detections')
        self.declare_parameter('use_class_names', True) # allows for human readable class names

        yolo_dir = self.get_parameter('yolo_dir').get_parameter_value().string_value
        weights = self.get_parameter('weights').get_parameter_value().string_value
        self.conf_thres = float(self.get_parameter('conf_thres').value)
        device_req = self.get_parameter('device').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.use_class_names = bool(self.get_parameter('use_class_names').value) # stor as instance for usability in other methods

        # selecting compute device for yolo, verify jetson
        if device_req == 'auto':
            device_str = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        else:
            device_str = device_req
        try:
            self.device = torch.device(device_str)
        except Exception as e:
            self.get_logger().warn(f"Failed to set device for YOLOv5: {device_str} was not recognized. Using CPU instead.")
            self.device = torch.device('cpu')
        

        # load yolo model
        self.get_logger().info(f"Loading YOLOv5 model from {yolo_dir} with weights {weights} on device {self.device}")
        self.model = torch.hub.load(yolo_dir, 'custom', path=weights, source='local', verbose=False)
        self.model.to(self.device)

        if hasattr(self.model, 'conf'): # different version of yolo allow for conf_thres manipulation, if we switch models this should allow us to set the conf level
            self.model.conf = self.conf_thres
        
        self.class_names = getattr(self.model, 'names', {})

        # qos
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            image_topic,
            self._on_image,
            qos
        )
        self.pub = self.create_publisher(
            Detection2DArray,
            output_topic,
            qos
        )

        self.get_logger().info(
            f'YOLO → Detection2DArray ready. Sub: {image_topic}  Pub: {output_topic}  '
            f'conf={self.conf_thres} device={self.device}'
        )

    def _on_image(self, msg: Image):
        # Convert ROS Image message to OpenCV format (BGR)
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed to convert image: {e}")
            return
    
        # inference timer, monitor performance
        t0 = time.time()
        try: 
            results = self.model(frame, size=640, conf=self.conf_thres)
        except TypeError: # if conf manipulation fails
            results = self.model(frame, size=640)
        except Exception as e:
            self.get_logger().error(f"YOLOv5 inference failed: {e}")
            return

        # detach and convert to numpy array, numpy is a cpu only library
        xyxy = results.xyxy[0].detach().cpu().numpy() if hasattr(results, 'xyxy') else np.zeros((0, 6))

        det_array = Detection2DArray() # Create a Detection2DArray message
        det_array.header = msg.header # Copy the header from the input image message(stamp, frame_id) into the detections array msg, tf transforms know what camera

        for x1, y1, x2, y2, conf, cls_id in xyxy:
            w = float(x2 - x1)
            h = float(y2 - y1)
            cx = float(x1 + w / 2)
            cy = float(y1 + h / 2)
            cls_id = int(cls_id)
            name = self.class_names.get(cls_id, str(cls_id))

            # individual frame by frame header, just in case we end up only pulling one frame for detection
            det = Detection2D()
            det.header = msg.header

            hyp = ObjectHypothesisWithPose()
            # if human readable class names then place if not just place the class id
            hyp.hypothesis.class_id = name if self.use_class_names else str(cls_id)
            hyp.hypothesis.score = float(conf)
            det.results.append(hyp)

            det.bbox = BoundingBox2D()
            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = w
            det.bbox.size_y = h

            det_array.detections.append(det)

        self.pub.publish(det_array)

        t1 = time.time()
        self.get_logger().debug(f"published {len(det_array.detections)} detections in {(t1 - t0)*1000:.1f} ms")

def main():
    rclpy.init()
    node = YoloV5Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()