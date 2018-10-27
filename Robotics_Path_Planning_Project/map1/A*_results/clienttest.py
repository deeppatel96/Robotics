
import rospy
from turtlebot_ctrl.srv import TurtleBotControl, TurtleBotControlResponse

def move_turtlebot_client(x,y,z):
	try:
		rospy.wait_for_service("turtlebot_control")
		turtlebot_control = rospy.ServiceProxy('turtlebot_control', TurtleBotControl)
		controlobj = TurtleBotControl()
		controlobj.x = float(x)
		controlobj.y = float(y)
		controlobj.z = float(z)
		response = turtlebot_control(controlobj)
		return response
	except rospy.ServiceException, e:
		print("Service Failed")

path = []
with open("result0.txt") as f:
	for line in f:
		int_list = [float(i) for i in line.split()]
		path.append(int_list)

	for i in range(0, len(path)):
		finish = 1
		print(path[i][0], path[i][1])
		if move_turtlebot_client(path[i][0], path[i][1], 0) == False:
			finish = 0
			break;

	if finish==0:
		print("Obstacle hit")
	else:
		print("Reached goal!")