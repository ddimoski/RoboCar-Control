#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from pynput.keyboard import Listener, Key

class KeyboardPublisher:

    def __init__(self):
        # logging the node start
        rospy.loginfo("Starting node keyboard_publisher")

        # Create a publisher to the keypress topic
        self.keyboard_publisher = rospy.Publisher('/keypress', String, queue_size=1)

        # Initialise the node
        rospy.init_node("keypress_publisher", anonymous=True)

    def publish_keypress(self, key_press):
        print(key_press.data);
        self.keyboard_publisher.publish(key_press)

    def on_press(self, key):
        pressed_key = String(data = str(key))
        #print(pressed_key.data)
        self.publish_keypress(pressed_key)

    def on_release(self, key):
        print('{0} release'.format(
            key))
        if key == Key.esc:
            # Stop listener
            return False

    def keyboard_listener(self):
        # Collect events until released
        with Listener(
                on_press=self.on_press,
                on_release=self.on_release) as listener:
            listener.join()


if __name__ == '__main__':
   keyboard_publisher = KeyboardPublisher()
   keyboard_publisher.keyboard_listener()

   try:
     rospy.spin()
   except rospy.ROSInterruptException:
     print("Stopping keyboard_publisher")
