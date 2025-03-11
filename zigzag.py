import rospy
from geometry_msgs.msg import Twist
import time

class ZigZagDrone:
    def __init__(self):
        rospy.init_node("zigzag_drone_movement", anonymous=True)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.vel_msg = Twist()
        self.num_moves = 6  # Number of zigzag motions
        self.move_distance = 2  # Distance between two points in meters

    def move_forward(self, distance=2):  # Default distance is 2 meters
        self.vel_msg.linear.x = 0.5  # Forward velocity in meters per second
        self.vel_msg.angular.z = 0.0
        # Calculate time needed to travel the desired distance
        time_to_move = distance / self.vel_msg.linear.x
        self.cmd_pub.publish(self.vel_msg)
        rospy.loginfo(f"Moving forward for {distance} meters.")
        time.sleep(time_to_move)  # Sleep to allow the drone to move forward
        self.stop()

    def turn_left(self, angle_duration=2):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.3  # Turn rate in radians per second
        self.cmd_pub.publish(self.vel_msg)
        rospy.loginfo("Turning left.")
        time.sleep(angle_duration)
        self.stop()

    def turn_right(self, angle_duration=2):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = -0.3
        self.cmd_pub.publish(self.vel_msg)
        rospy.loginfo("Turning right.")
        time.sleep(angle_duration)
        self.stop()

    def stop(self):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        self.cmd_pub.publish(self.vel_msg)
        time.sleep(1)  # Allow the drone to stop

    def zigzag_motion(self):
        for i in range(self.num_moves):
            # Move forward by the desired distance
            self.move_forward(self.move_distance)
            
            # Alternate between turning left and right based on the step
            if i % 2 == 0:
                self.turn_left()  # Turn left after moving forward
            else:
                self.turn_right()  # Turn right after moving forward

            rospy.loginfo(f"Zigzag Step {i+1} Completed")

if __name__ == "__main__":
    try:
        zigzag = ZigZagDrone()
        zigzag.zigzag_motion()
    except rospy.ROSInterruptException:
        pass

