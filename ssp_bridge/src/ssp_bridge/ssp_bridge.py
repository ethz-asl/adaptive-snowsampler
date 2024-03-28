#!/usr/bin/env python
import rospy
import serial
import threading
import time
from std_msgs.msg import Int8, Float64, UInt8MultiArray
from snowsampler_msgs.srv import (
    TakeMeasurement,
    Trigger,
    GetPosition,
    SetMaxSpeed,
    SetMeasurementDepth,
    TakeMeasurementResponse,
    TriggerResponse,
    GetPositionResponse,
    SetMaxSpeedResponse,
    SetMeasurementDepthResponse,
)


class SSPState:
    Error = 0
    Ready_To_Measure = 1
    Taking_Measurement = 2
    Stopped_No_Home = 3
    Going_Home = 4
    Moving = 5
    Position_Not_Reached = 6
    ENUM_LENGTH = 7


class SSPBridge:
    def __init__(self, port="/dev/ttySsp", baud_rate=115200):
        self.state_ = SSPState.Error
        self.position_ = 0.0
        self.port = port
        self.baud_rate = baud_rate
        self.serial = serial.Serial(port, baud_rate, timeout=1)
        self.read_thread = threading.Thread(target=self.read_serial)
        self.read_thread.daemon = True
        self.running = False
        self.start_serial()

        # Publishers
        self.state_publisher_ = rospy.Publisher("SSP/state", Int8, queue_size=100)
        self.position_publisher_ = rospy.Publisher(
            "SSP/position", Float64, queue_size=100
        )
        self.serial_publisher_ = rospy.Publisher(
            "serial_write", UInt8MultiArray, queue_size=100
        )

        # Services
        self.srv_take_measurement_ = rospy.Service(
            "SSP/take_measurement", TakeMeasurement, self.take_measurement_service
        )
        self.srv_stop_measurement_ = rospy.Service(
            "SSP/stop_measurement", Trigger, self.stop_measurement_service
        )
        self.srv_go_home_ = rospy.Service("SSP/go_home", Trigger, self.go_home_service)
        self.srv_get_position_ = rospy.Service(
            "SSP/get_position", GetPosition, self.get_position_service
        )
        self.srv_set_max_speed_ = rospy.Service(
            "SSP/set_max_speed", SetMaxSpeed, self.set_max_speed_service
        )
        self.srv_set_measurement_depth_ = rospy.Service(
            "SSP/set_measurement_depth",
            SetMeasurementDepth,
            self.set_measurement_depth_service,
        )

        # Serial Subscriber
        self.serial_subscriber_ = rospy.Subscriber(
            "serial_read", UInt8MultiArray, self.process_serial
        )
        rospy.loginfo("ssp_bridge sucessfuly started")

    def start_serial(self):
        """Starts the reading thread."""
        self.running = True
        self.read_thread.start()

    def stop_serial(self):
        """Stops the reading thread."""
        self.running = False
        self.serial.close()

    def write_serial(self, data):
        """Writes data to the serial device."""
        if self.running:
            self.serial.write(f"{data}\n".encode())
        # rospy.loginfo("Sent: " + data)

    def take_measurement_service(self, req):
        self.write_serial("take measurement:" + str(req.id))
        return TakeMeasurementResponse(success=True)

    def stop_measurement_service(self, req):
        self.write_serial("stop measurement")
        return TriggerResponse(success=True)

    def go_home_service(self, req):
        self.write_serial("go home")
        return TriggerResponse(success=True)

    def get_position_service(self, req):
        self.write_serial("get position")
        return GetPositionResponse(position=self.position_, success=True)

    def set_max_speed_service(self, req):
        self.write_serial("set max speed: " + str(req.data))
        return SetMaxSpeedResponse(success=True)

    def set_measurement_depth_service(self, req):
        self.write_serial("set measurement depth: " + str(req.data))
        return SetMeasurementDepthResponse(success=True)

    def read_serial(self):
        """Reads data from the serial device."""
        while self.running:
            if self.serial.in_waiting:
                data = self.serial.readline().decode().strip()
                if data:  # If data is not empty
                    # print(f"Received: {data}")
                    self.process_serial(data)
            time.sleep(0.1)

    def process_serial(self, out_str):

        # Find positions of the state and position in the string
        state_pos = out_str.find("State: ")
        position_pos = out_str.find("Position: ")

        if state_pos != -1 and position_pos != -1:
            try:
                # Extract state and position from the string
                state_str = out_str[
                    state_pos + 7 : position_pos - 2
                ]  # Adjust indices as needed
                position_str = out_str[position_pos + 10 :].replace(
                    "mm", ""
                )  # Remove 'mm' and extract position

                state_num = int(state_str)
                position = float(position_str)

                #rospy.loginfo("state_: %d", state_num)
                #rospy.loginfo("position_: %f", position)

                # Update state and position if valid
                if 0 <= state_num < SSPState.ENUM_LENGTH:
                    self.state_ = state_num
                    self.position_ = position

                    # Publish state and position
                    state_msg = Int8()
                    state_msg.data = state_num
                    self.state_publisher_.publish(state_msg)

                    position_msg = Float64()
                    position_msg.data = position
                    self.position_publisher_.publish(position_msg)

            except ValueError as e:
                rospy.logerr("Error processing serial data: %s", e)

    def on_shutdown(self):
        self.stop_serial()
        rospy.loginfo("Shutting down SSP bridge")


if __name__ == "__main__":
    try:
        rospy.init_node("ssp_bridge")
        ssp_bridge = SSPBridge()
        rospy.on_shutdown(ssp_bridge.on_shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
