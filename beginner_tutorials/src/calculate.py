import rospy
import scipy.optimize import fsolve
import numpy as np

def cal(data):
	def f(x):
		x1,x2,x3,x4=x[0],x[1],x[2],x[3]
		T1 = np.array([1,0,0,0], [0,1,0,0],[0,0,1,0],[0,0,0,1])
		T2 = np.array([1,0,0,0], [0,1,0,0],[0,0,1,0],[0,0,0,1])
		T3 = np.array([1,0,0,0], [0,1,0,0],[0,0,1,0],[0,0,0,1])
		T4 = np.array([1,0,0,0], [0,1,0,0],[0,0,1,0],[0,0,0,1])
		T5 = np.array([1,0,0,0], [0,1,0,0],[0,0,1,0],[0,0,0,1])
		T6 = np.array([1,0,0,0], [0,1,0,0],[0,0,1,0],[0,0,0,1])
		T7 = np.array([1,0,0,0], [0,1,0,0],[0,0,1,0],[0,0,0,1])
		T8 = np.array([1,0,0,0], [0,1,0,0],[0,0,1,0],[0,0,0,1])
	result = fsolve(f,[0.5,0.5,0.5,0.5])
	
def calculate():
	rospy.init_node('calculate', anonymous=True)
	rospy.Subscriber('', , cal)
	rospy.spin()
if _name_ == '_main':
	calculate()
