#!/usr/bin/env python  
import rospy
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool

class PointPub:
  def __init__(self):  
    rospy.on_shutdown(self.cleanup) # Call the cleanup function before finishing the node.  
    ###******* INIT PUBLISHERS *******###  
    rospy.Subscriber('GOAL_REACHED', Bool, self.get_goal_status)
    self.goal_pub = rospy.Publisher('GOAL', Point, queue_size=1)
    ###******* INIT CONSTANTS/VARIABLES *******### 
    self.goal_status = False 
    GOALS = [
        # (0.51, 2.08),
        (2.39, 4.77),
        (1.79, 3.57),
        (1.53, 1.30),
        (1.42, 0.21),
    ]
    GOAL_INDEX = 0
    GOAL = Point()
    GOAL.x, GOAL.y = GOALS[GOAL_INDEX]
    
    rate = rospy.Rate(1) # The rate of the while loop will be 50Hz 
    rospy.loginfo("Starting Message!")
    while not rospy.is_shutdown(): 
      if self.goal_status is False:
        rospy.loginfo("PUBLISHING: " + str(GOAL.x) + "," + str(GOAL.y))
        self.goal_pub.publish(GOAL)
      else:
        # Actualizar GOAL
        if GOAL_INDEX + 1 < len(GOALS):
          GOAL_INDEX += 1
          GOAL.x, GOAL.y = GOALS[GOAL_INDEX]
          self.goal_status = False
        else:
          rospy.loginfo("RUTINA COMPLETA")
          self.goal_status = True

      rate.sleep()

  def get_goal_status(self, msg=Bool()):
    self.goal_status = msg.data
  
  def cleanup(self):
    rospy.logwarn("FINISH MESAGE")

if __name__ == "__main__":   
    rospy.init_node('POINT_PUB') # Node Name
    try: PointPub()  # Class Name
    except rospy.ROSInterruptException:
        rospy.logwarn("EXECUTION COMPELTED SUCCESFULLY")

