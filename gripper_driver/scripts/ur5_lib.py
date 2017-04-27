#!/usr/bin/python2.7
"""
Author 	: Johan Kok Zhi Kang
Mail 	: JKOK005@e.ntu.edu.sg 

This is a custom motion planning library for the UR5. Motion planning is currently done using available algorithms from
OMPL. 
 
Dependencies:
1) ur_kin_py library	- python-UR kinematics wrapper class from https://github.com/gt-ros-pkg/ur_kin_py.git
2) boost numpy 
3) rospy
4) ros tf library 
5) numpy 
"""

import rospy 
import numpy as np
import actionlib
from math import pi
from std_msgs.msg import *
from trajectory_msgs.msg import *
from control_msgs.msg import *
from sensor_msgs.msg import *
from mario_utility import *
from gripper_driver.msg import *
from ur_kin_py.kin import Kinematics 
from tf import transformations as TF
from copy import copy

class VelocityProfile(object):
	def __init__(self, *args, **kwargs):
		self.max_angular_vel	= 0.8
		self.min_angular_vel	= 0.6
		super(VelocityProfile, self).__init__(*args, **kwargs)

	def __get_velocity_ramp(self, itr, total_points):
		if itr <= total_points /3:
			vel 		= (3 *float(itr) /total_points) *(self.max_angular_vel -self.min_angular_vel) + self.min_angular_vel
		elif itr >= total_points *2/3:
			vel 		= self.max_angular_vel -(3 *float(itr) /total_points -2) * (self.max_angular_vel -self.min_angular_vel)
		else:
			vel 		= self.max_angular_vel
		return vel

	def get_time_ramp_trajectory(self, del_theta, itr, total_points):
		velocity 			= self.__get_velocity_ramp(itr, total_points)
		time_interp 		= float(del_theta / velocity)
		return time_interp

class SubscribeToActionServer(VelocityProfile):
	def __init__(self, is_simulation, *args,**kwargs):
		if(is_simulation):
			# For gazebo
			rospy.loginfo("SubscribeToActionServer -> Running simulation in Gazebo")
			self.server_client 		= actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)	
		else:
			# Actual robot
			rospy.loginfo("SubscribeToActionServer -> Running on actual UR5")
			self.server_client 		= actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)

		if(not self.__establish_server_connection()):
			rospy.logerr("SubscribeToActionServer -> Unable to establish connection with ur5.")

		super(SubscribeToActionServer, self).__init__(*args, **kwargs)

	def __single_point_numpy_array_wrapper(self, raw_joint_space):
		# Function to convert a single  goal point of type list into numpy array for sending to action server
		if(isinstance(raw_joint_space[0], (float, int))):
			joint_space 		= [np.array(raw_joint_space)]		
		else:
			joint_space 		= raw_joint_space

		return joint_space

	def __frame_header_message(self):
		header 					= Header()
		header.seq 				= 1
		header.stamp			= rospy.Time.now()
		header.frame_id 		= "10"
		return header

	def __frame_single_point(self, joint_space):
		# Only 1 point to move to in joint space
		joint_space 			= self.__single_point_numpy_array_wrapper(joint_space)

		for instance in joint_space:
			single_point 			= instance
			single_point[0:5] 		*= -1 						# Some hacks to align Gazebo coordinate system with OpenRave
			point 					= JointTrajectoryPoint()
			point.positions 		= single_point.tolist()
			point.velocities 		= [0.005,0.005,0.005,0.005,0.005,0.005]
			point.accelerations 	= []
			point.time_from_start 	= rospy.Duration(5) 		# time from start must be in increasing order based on way point sequence
		return [point]

	def __frame_points_list(self, joint_space, total_points):
		point_list 				= []
		time_cumulated 			= 0.01
		no_of_points 			= total_points -1
		start 					= None
		end 					= None
		itr 					= 0

		if(total_points == 1):
			point_list  		+= self.__frame_single_point(joint_space)
		else:
			for each_joint in joint_space:								
				if(end == None):
					end 	= each_joint
				else:
					start 					= end
					end 					= each_joint
					itr 					+= 1

					end_modified 			= copy(end)
					end_modified[0:5] 		= end_modified[0:5] *-1 		# Some hacks to align Gazebo coordinate system with OpenRave

					point 					= JointTrajectoryPoint()
					point.positions 		= end_modified.tolist()
					point.velocities 		= [0.005,0.005,0.005,0.005,0.005,0.005]
					point.accelerations 	= []
					point.time_from_start 	= rospy.Duration(time_cumulated) 		# time from start must be in increasing order based on way point sequence

					del_theta 				= max(abs(end -start))
					time_cumulated			+= self.get_time_ramp_trajectory(del_theta=del_theta, itr=itr, total_points=no_of_points)
					point_list 				+= [point]
		return point_list

	def __frame_trajectory_message(self, joint_space, total_points):
		trajectory 				= JointTrajectory()
		trajectory.header 		= self.__frame_header_message()
		trajectory.joint_names 	= self.joint_names
		trajectory.points 		= self.__frame_points_list(joint_space, total_points)
		return trajectory

	def __frame_path_tolerance_message(self):
		path_tolerance 					= JointTolerance()
		path_tolerance.name 			= "path_tolerance"
		path_tolerance.position 		= 0.1
		path_tolerance.velocity 		= 0.1
		path_tolerance.acceleration 	= 0
		return [path_tolerance]

	def __frame_goal_tolerance_message(self):
		goal_tolerance 					= JointTolerance()
		goal_tolerance.name 			= "goal_tolerance"
		goal_tolerance.position 		= 0.1
		goal_tolerance.velocity 		= 0.1
		goal_tolerance.acceleration 	= 0
		return [goal_tolerance]

	def __frame_goal_message(self, joint_space, total_points):
		goal_message 				= FollowJointTrajectoryGoal()

		trajectory 					= self.__frame_trajectory_message(joint_space=joint_space, total_points=total_points)
		path_tolerance 				= self.__frame_path_tolerance_message()
		goal_tolerance 				= self.__frame_goal_tolerance_message()

		goal_message.trajectory 			= trajectory
		goal_message.path_tolerance			= path_tolerance
		goal_message.goal_tolerance			= goal_tolerance
		goal_message.goal_time_tolerance 	= rospy.Duration(5,0)
		return goal_message

	def __establish_server_connection(self):
		is_conn_success 		= False
		if(self.server_client.wait_for_server()):
			rospy.loginfo("SubscribeToActionServer -> Connected to action server")
			is_conn_success 	= True
		else:
			rospy.logerr("SubscribeToActionServer -> Unable to connect to action server")
		return is_conn_success

	def action_server_move_arm(self, joint_space, total_points):
		# Handles framing of message with timing given by the velocity profile and publishing to action server
		# Joint space is a list of np.arrays data type
		if(total_points == 0):
			rospy.logerr("SubscribeToActionServer -> No joint space coords detected.")
			return 
		goal_message 			= self.__frame_goal_message(joint_space, total_points)
		self.server_client.send_goal(goal_message)
		rospy.loginfo("SubscribeToActionServer -> Sending motion planning points to server")
		if(not self.server_client.wait_for_result(rospy.Duration(30,0))):
			rospy.logerr("SubscribeToActionServer -> Server took too long to respond with result.")
			self.server_client.cancel_goal()
		else:
			rospy.loginfo("SubscribeToActionServer -> Movement of arm complete")
		return

class GripperController(object):
	def __init__(self, *args, **kwargs):
		self.motor_command_pub 	= rospy.Publisher('/motor/command', motor_command, queue_size=10)
		self.pump_input_pub 	= rospy.Publisher('gripper/pump_input', Bool, queue_size=10)
		super(GripperController, self).__init__(*args, **kwargs)

	def __frame_pump_msg_and_publish(self, pump_input):
		msg 		= Bool()
		msg.data 	= pump_input
		self.pump_input_pub.publish(msg)
		return

	def on_pump(self):
		self.__frame_pump_msg_and_publish(1)
		return

	def off_pump(self):
		self.__frame_pump_msg_and_publish(0)
		return

	def __frame_gripper_msg_and_publish(self, read_angle, read_load, gripper_ready, gripper_open, gripper_close, gripper_standby):
		msg 				= motor_command()
		msg.read_angle 		= read_angle
		msg.read_load 		= read_load
		msg.gripper_ready 	= gripper_ready
		msg.gripper_open 	= gripper_open
		msg.gripper_close 	= gripper_close
		msg.gripper_standby = gripper_standby
		self.motor_command_pub.publish(msg)
		return

	def open_gripper(self):
		self.__frame_gripper_msg_and_publish(0,0,0,1,0,0)
		return

	def close_gripper(self):
		self.__frame_gripper_msg_and_publish(0,0,0,0,1,0)
		return

	def ready_gripper(self):
		self.__frame_gripper_msg_and_publish(0,0,1,0,0,0)
		return

	def standby_gripper(self):
		self.__frame_gripper_msg_and_publish(0,0,0,0,0,1)
		return

	def read_angle(self):
		self.__frame_gripper_msg_and_publish(1,0,0,0,0,0)
		return

	def read_load(self):
		self.__frame_gripper_msg_and_publish(0,1,0,0,0,0)
		return

class MarioKinematics(object):
	joint_lim_low 				= [-2*np.pi,-2*np.pi,-2*np.pi,-2*np.pi,-2*np.pi,-2*np.pi]			# Default joint limits.
	joint_lim_high 				= [-i for i in joint_lim_low]
	kin 						= Kinematics("ur5")

	def __init__(self, is_simulation, *args, **kwargs):
		# rospy.init_node('UR5_motion_planner', anonymous=True)
		# self.joint_names 				= ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
		
		self.__robot_joint_state 		= []
		if(is_simulation):
			rospy.Subscriber('/arm_controller/state', JointTrajectoryControllerState, self.__robot_joint_state_callback_simulation)
		else:
			rospy.Subscriber('/joint_states', JointState, self.__robot_joint_state_callback_actual)
		super(MarioKinematics, self).__init__(is_simulation, *args, **kwargs)

	def __unicode__(self):
		return """UR5 custom motion planning library"""
	
	def __robot_joint_state_callback_actual(self, data):
		self.__robot_joint_state 		= data.position
		return

	def __robot_joint_state_callback_simulation(self, data):
		actual_joint_state_msg_frame 	= data.actual
		self.__robot_joint_state  		= actual_joint_state_msg_frame.positions
		return

	@classmethod
	def change_joint_limits(cls, lim_low, lim_high):
		try:
			cls.__verify_joint_input(lim_low,lim_high)
			cls.joint_lim_low			= lim_low
			cls.joint_lim_high			= lim_high
		except AssertionError as e:
			print "Wrong input length or type for limits"
		except Exception as e:
			# Do not interrupt programme flow by breaking code
			print e	
		return

	@classmethod
	def __verify_joint_input(cls, lim_low,lim_high):	
		assert type(lim_low) is list and len(lim_low) == 6
		assert type(lim_high) is list and len(lim_high) == 6
		for i in range(6):
			if lim_low[i] > lim_high[i]:
				raise Exception('Please sort your high and low limits')
		return

	@classmethod
	def cartesian_from_joint(cls, joint_vals):
		# Returns a list of [roll, pitch, yaw, X, Y, Z] from joint values
		# joint_vals either be a list of multiple joint sets or a single joint set
		if(all(isinstance(i, list) for i in joint_vals)):
			lst 				= []
			for each_joint_val in joint_vals:
				fk_sol	 		= cls.kin.forward(q=each_joint_val)
				lst 			+= [cls.__get_cartesian_from_matrix(fk_sol)]
		else:
			fk_sol	 			= cls.kin.forward(q=joint_vals)
			lst 				= cls.__get_cartesian_from_matrix(fk_sol)
		return lst

	@classmethod
	def __get_cartesian_from_matrix(cls, h_matrix):
		# Returns a list of [roll, pitch, yaw, X, Y, Z]
		euler_from_matrix					= list(TF.euler_from_matrix(h_matrix))
		translation_from_matrix				= list(TF.translation_from_matrix(h_matrix))
		return euler_from_matrix + translation_from_matrix

	def get_joint_sol_from_bin_grasping(self, obj_label, grasp_results, grasp_type):
		# obj_label is the identifier for bin or tote location
		suction_method 												= self.__get_gripper_suction_method(grasp_type)
		roll, pitch, yaw, item_coord_X, item_coord_Y, item_coord_Z 	= grasp_results
		# roll = -pi/2
		# pitch = 0
		# yaw = 0
		# item_coord_X, item_coord_Y, item_coord_Z 	= grasp_results
		gripper_coord_X, gripper_coord_Y, gripper_coord_Z, _		= RobotToNewShelfTransformation.get_item_coord_from_obj_to_robot(obj_label, item_coord_X, item_coord_Y, item_coord_Z)
		gripper_offset_X, gripper_offset_Y, gripper_offset_Z 		= suction_method.gripper_frame_to_end_effector_displacement(roll, pitch, yaw)
		
		base_coord_X 	= gripper_coord_X -gripper_offset_X
		base_coord_Y 	= gripper_coord_Y -gripper_offset_Y 		
		base_coord_Z 	= gripper_coord_Z -gripper_offset_Z
		cartesian		= [roll, pitch, yaw, base_coord_X, base_coord_Y, base_coord_Z]

		candidate_sols 			= self.cartesian_to_ik(cartesian=cartesian)
		selected_ik_sol 		= self.kin.get_closest_joint_sol(current_joint=self.get_robot_joint_state(), candidate_sols=candidate_sols)		
		selected_ik_sol[0:5]	*= -1 			# HACKS FOR GAZEBO

		return selected_ik_sol

	def get_joint_sol_from_tote_grasping(self, obj_label, grasp_results, grasp_type):
		# obj_label is the identifier for bin or tote location
		suction_method 												= self.__get_gripper_suction_method(grasp_type)
		roll, pitch, yaw, item_coord_X, item_coord_Y, item_coord_Z 	= grasp_results
		base_coord_X, base_coord_Y, base_coord_Z, _					= RobotToToteTransformation.get_item_coord_from_obj_to_robot(obj_label, item_coord_X, item_coord_Y, item_coord_Z)
		gripper_offset_X, gripper_offset_Y, gripper_offset_Z 		= suction_method.gripper_frame_to_end_effector_displacement(roll, pitch, yaw)
		
		base_coord_X 	= gripper_coord_X -gripper_offset_X
		base_coord_Y 	= gripper_coord_Y -gripper_offset_Y 		
		base_coord_Z 	= gripper_coord_Z -gripper_offset_Z
		cartesian		= [roll, pitch, yaw, base_coord_X, base_coord_Y, base_coord_Z]
		return self.cartesian_to_ik(cartesian=cartesian)

	def __get_gripper_suction_method(self, grasp_type):
		# 1 - Front suction method. 0 - Side suction method.
		if(grasp_type):
			return GripperFrontSuctionOffset
		else:
			return GripperSideSuctionOffset

	def cartesian_to_ik(self, cartesian):
		# Cartesian coordinates of [roll,pitch,yaw,X,Y,Z] to ik solutions
		# Returns all possible IK solutions within joint limits if single is False
		# Returns best IK solution within joint limit if single is True
		roll,pitch,yaw,X,Y,Z 				= cartesian
		matrix_from_euler					= TF.euler_matrix(roll,pitch,yaw)
		matrix_from_translation				= TF.translation_matrix([X,Y,Z])
		matrix_from_cartesian 				= TF.concatenate_matrices(matrix_from_translation, matrix_from_euler)
		return self.__get_ik_from_matrix(matrix_from_cartesian)

	def __get_ik_from_matrix(self, h_matrix):
		# Gets all possible IK solutions from homogeneous matrix that satisfies joint limits defined
		# If no possible IK solutions for given point, throws AssertionError
		ik_sols								= self.kin.inverse_all(x=h_matrix)
		ik_sols_cleaned						= self.__assert_joint_limits(ik_sols)
		assert len(ik_sols_cleaned) is not 0
		return ik_sols_cleaned

	def __assert_joint_limits(self, ik_sols):
		# Asserts that the given joint space coordinate falls within the allowable joint limits or throw AssertionError
		ik_sols_cleaned 					= []
		for each_ik_sol in ik_sols:
			if self.__solution_within_joint_limits(each_ik_sol):
				ik_sols_cleaned.append(each_ik_sol)
		return ik_sols_cleaned

	def __solution_within_joint_limits(self, each_ik_sol):
		# If not within joint limit, reject each_ik_sol
		length 								= len(each_ik_sol)
		for i in range(length):
			# Check if joint limit has been exceed for each UR joint
			if each_ik_sol[i] < self.joint_lim_low[i] or each_ik_sol[i] > self.joint_lim_high[i]:
				return False
		return True

	def get_joint_space_from_delta_robot_frame(self, axis, delta_dist):
		def get_goal_cartesian_from_delta_dist(current_cartesian, axis, delta_dist):
			roll, pitch, yaw, x, y, z 			= current_cartesian
			default_roll_offser 				= -pi/2 				# offset hardcoded to make the gripper's axis align with gazebo's axis
			roll 								-= default_roll_offser
			matrix_from_euler					= TF.euler_matrix(roll,pitch,yaw)
			axis_translation_list 				= get_axis_translation_list(axis=axis, delta_dist=delta_dist)
			delta_matrix						= TF.translation_matrix(axis_translation_list)
			delta_goal_matrix_base_frame 		= np.dot(matrix_from_euler, delta_matrix)
			delta_goal_translation_base_frame 	= TF.translation_from_matrix(delta_goal_matrix_base_frame)
			final_goal_base_frame 				= delta_goal_translation_base_frame + [x,y,z]
			return [roll, pitch, yaw] + final_goal_base_frame.tolist()

		def get_axis_translation_list(axis, delta_dist):
			# Only move along the axis of the robot frame
			if axis == 'x':
				axis_translation_list 	= [delta_dist, 0, 0]
			elif axis == 'y':
				axis_translation_list 	= [0, delta_dist, 0]
			elif axis == 'z':
				axis_translation_list 	= [0, 0, delta_dist]
			else:
				raise Exception('MarioKinematics -> get_axis_translation_list -> axis given is not x,y,z')
			return axis_translation_list

		def plan_joint_way_points(current_cartesian, goal_cartesian):
			way_points 				= PoseInterpolator.plan_to_cartesian_linear(current_cartesian, goal_cartesian)
			joint_way_points 		= []
			current_joint 			= self.get_robot_joint_state()
			try:
				for pts in way_points:
					candidate_sols 		= self.cartesian_to_ik(cartesian=pts)
					ik_point 			= self.kin.get_closest_joint_sol(current_joint=current_joint, candidate_sols=candidate_sols)
					current_joint 		= copy(ik_point)
					ik_point[0:5] 		*= -1 							# HACKS to convert values to Gazebo referene frame
					joint_way_points.append(ik_point)
				return joint_way_points
			except AssertionError:
				raise AssertionError("No possible IK solutions found for coordinate: {0}".format(pts))

		# Moves the robot by an axis by a delta distance
		current_cartesian 			= self.get_robot_cartesian_state()
		goal_cartesian 				= get_goal_cartesian_from_delta_dist(current_cartesian, axis, delta_dist)
		return plan_joint_way_points(current_cartesian, goal_cartesian)

	def get_robot_joint_state(self):
		# Current joint state of Mario
		return list(self.__robot_joint_state)

	def get_robot_cartesian_state(self):
		robot_joint_state 		= self.get_robot_joint_state()
		return self.cartesian_from_joint(robot_joint_state)

class MarioFullSystem(MarioKinematics, SubscribeToActionServer, GripperController):
	def __init__(self, is_simulation, *args, **kwargs):
		rospy.init_node('Mario_stowing', anonymous=True)
		self.joint_names 		= ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
		super(MarioFullSystem, self).__init__(is_simulation, *args, **kwargs)

if __name__ == "__main__":
	is_simulation	= True
	mario 			= MarioFullSystem(is_simulation)

	grasp_result 	= [0, 0.0, -1.5707963267948966, 0.303, 0.185, 0.1155]		# roll / pitch / yaw / X / Y / Z
	obj_label 		= 'ready_bin_E'
	grasp_type 		= 0 				# Side suction

	selected_base_coord 		= mario.get_joint_sol_from_bin_grasping(obj_label, grasp_result, grasp_type)
	# selected_base_coord 		= base_coord[0]
	# selected_base_coord[0:5] 	*= -1 				# Some hacks to translate to Gazebo frame of reference

	# selected_base_coord 		= mario.get_joint_space_from_delta_robot_frame('z', 0.03)
	# selected_base_coord 		= [0.2687414328204554, 0.9837163130389612, -2.213135242462158, 1.3010690847979944, -1.2809619903564453, -1.5690296331988733]
	mario.action_server_move_arm(joint_space=selected_base_coord, total_points=1)