#!/usr/bin/env python3
import rospy
from led_msgs.srv import SetLEDs
from led_msgs.msg import LEDStateArray, LEDState
import json
import sys

rospy.init_node("gzweb_leds")

r = rospy.Rate(8)
while not rospy.is_shutdown():
  ros_leds = rospy.wait_for_message('led/state', LEDStateArray, timeout=10)
  leds = []
  for ros_led in ros_leds.leds:
    leds.append({
        "id":
        ros_led.index,
        "color": [
            round(ros_led.r / 255., 3),
            round(ros_led.g / 255., 3),
            round(ros_led.b / 255., 3),
            1,
        ]
    })

  sys.stdout.write(
      json.dumps({
          "op": "publish",
          "topic": "~/leds",
          "msg": leds
      }))
  sys.stdout.flush()

  r.sleep()