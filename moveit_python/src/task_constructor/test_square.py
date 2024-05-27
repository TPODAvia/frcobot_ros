#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_python import MoveGroupInterface
from moveit_python import PickPlaceInterface
from moveit_python import PlanningSceneInterface
import time
import sys
from std_msgs.msg import Float64
import moveit_commander

rospy.init_node('move_square_node')

bot = moveit_commander.RobotCommander()
if not bot.get_group_names()[0] == f"fr10_arm":
    print(f"Wrong robot name: fr10")
    sys.exit()

scene = PlanningSceneInterface("/base_link")
move_group_interface = MoveGroupInterface(group="fr10_arm", frame="world")
pick_place_interface = PickPlaceInterface(group="fr10_arm", ee_group="gripper")

pub = rospy.Publisher('/rh_p12_rn_position/command', Float64, queue_size=10)

def spawn_obj(x, y, z):
    scene.addBox("hello_box", 0.05, 0.05, 0.05, x, y, z, use_service=True)

def attach_obj():
    scene.attachBox("hello_box", 0.05, 0.05, 0.05, 0, 0, 0, link_name="tf_end")
        
def remove_obj_all():
    scene.clear()

def remove_collis_all():
    for name in scene.getKnownCollisionObjects():
        print("Removing %s" % name)
        scene.removeCollisionObject(name, use_service=False)
    scene.waitForSync()

def remove_collision(name):
    scene.removeCollisionObject(name)

def remove_attached(name):
    scene.removeAttachedObject(name)

def gripper_open():
    msg = Float64()
    msg.data = 0.0
    print(f"task_executer.py: Open: {msg.data}")
    pub.publish(msg)

def gripper_close(value):
    msg = Float64()
    if value < 0:
        msg.data = 0.68 # set to max
    else:
        msg.data = value
    print(f"task_executer.py: Close: {msg.data}")
    pub.publish(msg)

def move_to_pose(x, y, z):
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = 1.0
    result = move_group_interface.moveToPose(pose, gripper_frame="tf_end", tolerance=0.01, wait=True)
    if result.error_code.val < 1:
        print(f"task_executer.py Error: {result.error_code}")
        sys.exit()

def task():
    # Define the square corners
    dis_x = 1
    square_corners = [
        (dis_x + 0.5, 0.5, 0.2),
        (dis_x + 0.5, -0.5, 0.2),
        (dis_x - 0.5, -0.5, 0.2),
        (dis_x - 0.5, 0.5, 0.2)
    ]

    remove_attached("hello_box")
    remove_obj_all()
    time.sleep(5)

    # Move to each corner of the square
    for (x, y, z) in square_corners:
        print(f"Moving to corner: x={x}, y={y}, z={z}")
        move_to_pose(x, y, z)
        time.sleep(5)

    remove_obj_all()
    print("task_executer.py: Finish!")

if __name__ == '__main__':
    task()
