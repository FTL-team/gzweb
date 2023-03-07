#!/usr/bin/env python3
import rospy
from led_msgs.srv import SetLEDs
from led_msgs.msg import LEDStateArray, LEDState
import json
import sys
from threading import Lock

rospy.init_node("gzweb_leds")

lock: Lock = Lock()
ros_leds = None


def callback(data):
  global ros_leds
  lock.acquire()
  ros_leds = data
  lock.release()

rospy.Subscriber("led/state", LEDStateArray, callback)

r = rospy.Rate(8)
while not rospy.is_shutdown():
  lock.acquire()
  if ros_leds is not None:
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

  lock.release()
  r.sleep()