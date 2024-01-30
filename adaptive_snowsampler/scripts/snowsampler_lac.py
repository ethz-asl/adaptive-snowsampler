#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import numpy as np
import time
from lac import LAC
from snowsampler_msgs.srv import SetAngle


class snowsampler_lac(Node):
    def __init__(self):
        super().__init__("snowsampler_lac")
        self.stroke = 150
        self.stroke_min = 5  # derived in colab ipynb | exact lower limit: 11.3
        self.stroke_max = 135  # derived in colab ipynb | exact upper limit: 129.34
        self.topic = "snowsampler/landing_leg_angle"

        # Publisher for the current angle
        self.angle_publisher = self.create_publisher(Float64, self.topic, 10)

        # Service for setting the desired angle
        self.srv = self.create_service(SetAngle, "snowsampler/set_landing_leg_angle", self.set_angle_callback)

        # Initialize the LAC instance
        self.lac = LAC(stroke=self.stroke)
        self.lac.set_retract_limit(self.stroke_min)
        self.lac.set_extend_limit(self.stroke_max)
        # Start the node's main loop
        self.timer = self.create_timer(0.1, self.publish_current_angle)
        self.get_logger().info("snowsampler_lac node has been started")

    def set_angle_callback(self, request, response):
        self.timer.cancel()
        # Call the set_position function of the LAC class
        l_act = self.angle_to_length(request.angle)
        resp = self.lac.set_position(l_act)
        timeout = 10 # seconds
        start_time = time.time()
        while np.abs(self.current_angle - request.angle) > 0.1:
            # Timeout if the LAC is not able to reach the desired position
            if time.time() - start_time > timeout:
                response.success = False
                return response

            resp = self.lac.set_position(l_act)
            self.current_angle = self.length_to_angle(resp[1] / 1023 * self.stroke)
            msg = Float64()
            msg.data = self.current_angle
            self.angle_publisher.publish(msg)
        # TODO: validate until the LAC is around the desired position
        self.timer = self.create_timer(0.1, self.publish_current_angle)
        response.success = True
        return response

    def publish_current_angle(self):
        # Get the current feedback from the LAC class
        feedback = self.lac.get_feedback()
        current_length = feedback / 1023 * self.stroke
        self.current_angle = self.length_to_angle(current_length)
        # Publish the current angle
        msg = Float64()
        msg.data = self.current_angle
        self.angle_publisher.publish(msg)

    def angle_to_length(self, angle):  # angle in degrees, length in mm
        # derived LSQ fit, l_act[mm] = m*phi[°] + c
        m = 2.64
        c = 12.45
        length = m * angle + c
        return length

    def length_to_angle(self, length):  # angle in degrees, length in mm
        # derived LSQ fit, phi[°] = m*l_act[mm] + c
        m = 2.64
        c = 12.45
        angle = (length - c) / m
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = snowsampler_lac()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
