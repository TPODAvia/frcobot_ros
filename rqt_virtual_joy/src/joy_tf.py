#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Joy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math

class JoyToTf:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('joy_to_tf_node', anonymous=True)

        self.map_tf = rospy.get_param('~map_tf', 'map')
        self.interactive_tf_base = rospy.get_param('~interactive_tf_base', 'base_link')
        self.interactive_tf = rospy.get_param('~interactive_tf', 'tf_end')
        self.joystick_tf = rospy.get_param('~joystick_tf', 'joystick_tf')

        # Initialize translation and rotation
        self.joystick_translation = [0.0, 0.0, 0.0]  # x, y, z
        self.joystick_translation_past = [0.0, 0.0, 0.0]
        self.rotation = [0.0, 0.0, 0.0]     # roll, pitch, yaw (in radians)
        self.map_translation_past = [0.0, 0.0, 0.0]

        # Store the previous time for integration
        self.last_time = rospy.Time.now()

        # Create a tf2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Subscribe to the /joy topic
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Create a tf2 buffer and listener to listen to the map -> base_link transform
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def joy_callback(self, joy_msg):
        # Get the current time and calculate the time delta for integration
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # Get the axes data (structured as velocity in x, y, z, roll, pitch, yaw rates)
        self.joystick_translation[0] += joy_msg.axes[0] * dt  # Velocity X integrated to position X
        self.joystick_translation[1] += joy_msg.axes[1] * dt  # Velocity Y integrated to position Y
        self.joystick_translation[2] += joy_msg.axes[2] * dt  # Velocity Z integrated to position Z

        # Integrate rotational velocities (roll, pitch, yaw rates) to update the rotation angles
        self.rotation[2] += joy_msg.axes[3] * dt     # Roll velocity to roll angle
        self.rotation[0] += joy_msg.axes[4] * dt     # Pitch velocity to pitch angle
        self.rotation[1] += joy_msg.axes[5] * dt     # Yaw velocity to yaw angle

        # Reset translation coordinates if button 1 is pressed
        if joy_msg.buttons[1] == 1:
            self.joystick_translation = [self.joystick_translation_past[0], self.joystick_translation_past[1], self.joystick_translation_past[2]]

        # Reset rotation coordinates if button 2 is pressed
        if joy_msg.buttons[0] == 1:
            self.rotation = [0.0, 0.0, 0.0]

        # Update the last_time to current_time for the next integration
        self.last_time = current_time

    def broadcast_transform(self):


        try:
            # Get the transform from map to base_link
            trans = self.tf_buffer.lookup_transform(self.interactive_tf_base, self.interactive_tf, rospy.Time(0), rospy.Duration(0.1))

            # Get the translation and rotation from the transform

            # Calculate the distance between current position and map -> base_link
            map_translation = trans.transform.translation
            translation_diff = math.sqrt((self.map_translation_past[0] - map_translation.x)**2 +
                                         (self.map_translation_past[1] - map_translation.y)**2 +
                                         (self.map_translation_past[2] - map_translation.z)**2)


            self.map_translation_past[0] = map_translation.x
            self.map_translation_past[1] = map_translation.y
            self.map_translation_past[2] = map_translation.z

            # # Convert quaternion to Euler angles to calculate rotation difference
            # map_rotation = trans.transform.rotation
            # map_euler = euler_from_quaternion([map_rotation.x, map_rotation.y, map_rotation.z, map_rotation.w])
            # rotation_diff = math.sqrt((self.rotation[0] - map_euler[0])**2 +
            #                           (self.rotation[1] - map_euler[1])**2 +
            #                           (self.rotation[2] - map_euler[2])**2)

            # If the translation or rotation difference exceeds 0.001, update the transform
            if translation_diff > 0.001: # or rotation_diff > 0.001:

                trans = self.tf_buffer.lookup_transform(self.map_tf, self.interactive_tf, rospy.Time(0), rospy.Duration(0.1))
                map_translation = trans.transform.translation

                self.joystick_translation[0] = map_translation.x
                self.joystick_translation[1] = map_translation.y
                self.joystick_translation[2] = map_translation.z

                self.joystick_translation_past[0] = map_translation.x
                self.joystick_translation_past[1] = map_translation.y
                self.joystick_translation_past[2] = map_translation.z

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            translation_diff = 0
            # rospy.logwarn("Could not get transform: %s", str(e))


        # print(translation_diff)
        # Create a TransformStamped message
        t = geometry_msgs.msg.TransformStamped()

        # Set the frame ids
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.map_tf
        t.child_frame_id = self.joystick_tf

        # print(f"{self.joystick_translation[0]}  {self.rotation[0]}")

        # Set the translation part
        t.transform.translation.x = self.joystick_translation[0]
        t.transform.translation.y = self.joystick_translation[1]
        t.transform.translation.z = self.joystick_translation[2]

        # Convert the integrated roll, pitch, yaw to a quaternion
        quat = quaternion_from_euler(self.rotation[0], self.rotation[1], self.rotation[2])

        # Set the rotation part (quaternion)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Send the transform
        self.tf_broadcaster.sendTransform(t)

if __name__ == '__main__':
    try:
        # Instantiate the JoyToTf class
        joy_to_tf = JoyToTf()

        # Run the broadcasting loop
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            joy_to_tf.broadcast_transform()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
