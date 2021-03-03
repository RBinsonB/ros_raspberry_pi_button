# coding=utf-8
import rospy
import RPi.GPIO as GPIO

from std_msgs.msg import Bool

class ButtonHandler(object):
    def __init__(self, topic_id, input_pin):
        self.status_msg = Bool()

        # Setup publisher
        self.button_pub = rospy.Publisher(topic_id, String, latch=True, queue_size=1)

        # Setup GPIO callback
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(input_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.add_event_detect(input_pin, GPIO.BOTH, callback=self.button_callback)

    # Button callback, call when button is changing state
    def button_callback(self, channel):
        if GPIO.input(channel) == GPIO.HIGH:
            self.status_msg.data = True
        else:
            self.status_msg.data = False

        self.button_pub.publish()


class ButtonBox():
    def __init__(self):
    self.button_box_dict = {}

    # Get parameters from ROS server
    self.button_definition = rospy.get_param("~", {})

    # Check params
    for button in self.button_definition.keys():
        # For every button, a dictionnary with topic_id and input pin is needed
        if ('topic_id' in self.button_definition[button].keys()) and ('input_pin' in self.button_definition[button].keys()):
            # Create the button handler object
            topic_id = self.button_definition[button]['topic_id']
            input_pin = self.button_definition[button]['input_pin']
            self.button_box_dict.update({str(button): ButtonHandler(topic_id, input_pin)})
            # Log about button
            rospy.loginfo("%s: connecting button on GPIO %i to topic %s%s" % (rospy.get_name(), input_pin, rospy.get_namespace(), topic_id))


#==============================
#             Main
#==============================
if __name__ == '__main__':
    rospy.init_node('button_box_node',anonymous=False)
    try:
        button_box = ButtonBox()
        rospy.spin()
    except ValueError as e:
        rospy.logerr("%s: %s" %(rospy.get_name(), e))
        sys.exit(0)
    