#! /usr/bin/env python3

import rospy
import socket
import smtplib, ssl

from fetch_driver_msgs.msg import RobotState
from fetch_core_msgs.msg import FetchcoreErrorStatus
from fetch_sound_msgs.msg import PlaySoundActionGoal
from geometry_msgs.msg import TransformStamped


class StateTracker():

    def __init__(self, robot_state_sub_topic, robot_error_sub_topic, robot_state_pub_topic, robot_sound_pub_topic, robot_tf_sub_topic):
        # create the /robot_state subscriber
        self.state_sub = rospy.Subscriber(robot_state_sub_topic, RobotState, self.state_callback)
        # create the /error_status_requests subscriber
        self.error_sub = rospy.Subscriber(robot_error_sub_topic, FetchcoreErrorStatus, self.error_callback)

        # create the /transforms/map_to_base_link subscriber
        self.tf_sub = rospy.Subscriber(robot_tf_sub_topic, TransformStamped, self.tf_callback)

        # create the stand in publisher
        self.state_pub = rospy.Publisher(robot_state_pub_topic, RobotState, queue_size=10)
        # create the sound publisher
        self.sound_pub = rospy.Publisher(robot_sound_pub_topic, PlaySoundActionGoal, queue_size=10)

        # set other variables
        self.hostname = socket.gethostname()
        self.runstopped_prev = False
        self.runstopped_curr = False
        self.dict = self.init_dictionary()
        self.translation = [None, None, None]


    def state_callback(self, msg):

        # create the objects
        state = RobotState()
        sound = PlaySoundActionGoal()

        # set values for fields
        self.runstopped_curr = msg.runstopped
		sound.goal.filename = '/root/catkin_ws/src/fetchcore/server/fetchcore_server/api/migrations/resources/sounds/chirp.wav'
        sound.goal.volume = 10.0

        # update the dictionary accordingly
        self.update_dictionary(msg, True)

        # play a sound when the e-stop is turned off
        if self.runstopped_prev == True and self.runstopped_curr == False:
            self.sound_pub.publish(sound)

        self.runstopped_prev = self.runstopped_curr

       # print(msg.runstopped)


    def error_callback(self, msg):

        if msg.type is not None:
           # take care of latched errors
           # there should be no difference between the values
            if int(rospy.get_time()) - msg.header.stamp.secs <= 1:
                self.update_dictionary(msg, False)


    def tf_callback(self, msg):
        # get the current position of the robot relative to the map's origin
        self.translation[0] = msg.transform.translation.x
        self.translation[1] = msg.transform.translation.y
        self.translation[2] = msg.transform.translation.z
        return


    def init_dictionary(self):
        # might need to be hardcoded which is :(
        dict = {'entry0': {'name': 'Runstopped', 'error_val': True, 'status': False, 'curr_error_status': False}, 'entry1':{'name': 'RealSense Camera', 'error_val': None, 'status': 'Running', 'curr_error_status': False} }
        return dict


    def update_dictionary(self, msg, is_state_value):
        if is_state_value == True:
            if msg.runstopped == self.dict['entry0']['error_val']:
                if self.dict['entry0']['curr_error_status'] != True:
                    self.dict['entry0']['curr_error_status'] = True
                    self.dict['entry0']['status'] = True
                    self.send_email(True)
            elif msg.runstopped != self.dict['entry0']['error_val']:
                if self.dict['entry0']['curr_error_status'] != False:
                    self.dict['entry0']['curr_error_status'] = False
                    self.dict['entry0']['status'] = False
                    self.send_email(False)

        else:
            if msg.type == 'CAMERA_ERROR':
                if self.dict['entry1']['curr_error_status'] != True:
                    self.dict['entry1']['curr_error_status'] = True
                    self.dict['entry0']['status'] = 'Not running'
                    self.send_email(True)
            elif msg.type != 'CAMERA_ERROR':
                if self.dict['entry1']['curr_error_status'] != False:
                    self.dict['entry1']['curr_error_status'] = False
                    self.dict['entry0']['status'] = 'Running'
                    self.send_email(False)

        return


    def build_message(self, is_broken):
        # initialize entire message string
        if is_broken == True:
            message = "Subject: ROBOT ERROR ALERT\n\n"
        else:
            message = "Subject: FIXED ROBOT ERROR\n\n"

        # initialize the error and non-error strings
        error_list = ""
        other_info = ""

        # add things to the error_list and other_info here
        for key, value in self.dict.items():
            if value['curr_error_status'] == True:
                error_list += "\t" + str(value['name']) + ": " + str(value['status']) + "\n"
            elif value['curr_error_status'] == False:
				other_info += "\t" + str(value['name']) + ": " + str(value['status']) + "\n"

        if error_list == "":
            error_list = "\tNone\n"

        if other_info == "":
            other_info = "\tNone\n"

        # put together message
        message += "ROBOT NAME: " + self.hostname + "\n\n"

        message += "KNOWN ERRORS\n" + error_list + "\n"

        message += "ROBOT LOCATION (relative to map origin)\n\t" + str(self.translation) + "\n\n"

        message += "OTHER INFORMATION\n" + other_info

        return message


    def send_email(self, is_broken):
        port = 587  # For starttls
		smtp_server = "smtp.gmail.com"
        sender_email = "anna.wong316@gmail.com"
        receiver_email = "anna.elise781@gmail.com"
        password = 'fsscacynxadzcofa'
        message = self.build_message(is_broken)

        context = ssl.create_default_context()
        with smtplib.SMTP(smtp_server, port) as server:
            server.ehlo()  # Can be omitted
            server.starttls(context=context)
            server.ehlo()  # Can be omitted
            server.login(sender_email, password)
            server.sendmail(sender_email, receiver_email, message)


if __name__ == '__main__':
    # initialize the node
    rospy.init_node('robot_state_tracker', anonymous=True)

    # define the publisher/subscriber topics
    robot_state_sub_topic = '/robot_state'
    robot_error_sub_topic = '/fetchcore/error_status_requests'
    robot_state_pub_topic = '/sender_of_stuff' # TODO: what should this actually be
    robot_sound_pub_topic = '/fetchcore/sound/goal'
    robot_tf_sub_topic = '/transforms/map_to_base_link'

    # initialize the class
    state_tracker = StateTracker(robot_state_sub_topic, robot_error_sub_topic, robot_state_pub_topic, robot_sound_pub_topic, robot_tf_sub_topic)

    # keep the program from exiting before the node is stopped
    rospy.spin()

