## comands for testing the services manually:

**take measurement:**

`ros2 service call /SSP/take_measurement snowsampler_msgs/srv/Trigger`

**stop measurement:**

`ros2 service call /SSP/stop_measurement snowsampler_msgs/srv/Trigger`

**go home:**

`ros2 service call /SSP/go_home snowsampler_msgs/srv/Trigger`

**get position:**

`ros2 service call /SSP/get_position snowsampler_msgs/srv/GetPosition`

**set max speed:**

`ros2 service call /SSP/set_max_speed snowsampler_msgs/srv/SetMaxSpeed '{"data":10}'`