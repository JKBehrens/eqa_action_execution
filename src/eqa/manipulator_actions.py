import copy
import math
import os
import sys
import time

from geometry_msgs.msg import Quaternion, PoseStamped, Point
from kortex_driver.msg import Finger, GripperMode, TwistCommand, BaseCyclic_Feedback, CartesianReferenceFrame, \
    JointSpeed
from kortex_driver.srv import SendGripperCommand, SendGripperCommandRequest, SendTwistCommand, SendTwistCommandRequest, \
    SendTwistCommandResponse, SendJointSpeedsCommand, SendJointSpeedsCommandRequest
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface, SolidPrimitive
import rosnode
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from action_classes import myObject

import inspect

# def get_class_that_defined_method(meth):
#     # meth.im_class
#     # args = inspect.getcallargs(meth)
#     for cls in inspect.getmro(meth.im_class):
#         if meth.__name__ in cls.__dict__:
#             return cls
#     return None
#
# def with_planning_scene(func):
#     ManipulatorActions_class = get_class_that_defined_method(func)
#     def wrapper():
#         ManipulatorActions_class.sc.remove_world_object()
#         ManipulatorActions_class.add_table_plane()
#         ManipulatorActions_class.avoid(*ManipulatorActions.scene_objects)
#         time.sleep(0.3)
#         func()
#
#     return wrapper()
#
# def without_planning_scene(func):
#     ManipulatorActions_class = get_class_that_defined_method(func)
#     def wrapper():
#         ManipulatorActions_class.sc.remove_world_object()
#         ManipulatorActions_class.add_table_plane()
#         time.sleep(0.3)
#         func()
#
#     return wrapper()

class ManipulatorActions(object):
    mg = None  # type: MoveGroupCommander
    sc = None  # type: PlanningSceneInterface
    rc = None  # type: RobotCommander

    scene_objects = []

    send_gripper_command_full_name = '/' + 'my_gen3' + '/base/send_gripper_command'
    send_gripper_command_srv = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

    cart_vel_control_srv_full_name = '/' + 'my_gen3' + '/base/send_twist_command'
    cart_vel_control_srv = rospy.ServiceProxy(cart_vel_control_srv_full_name, SendTwistCommand)

    joint_vel_control_srv_full_name = '/' + 'my_gen3' + '/base/send_joint_speeds_command'
    joint_vel_control_srv = rospy.ServiceProxy(joint_vel_control_srv_full_name, SendJointSpeedsCommand)

    UP = 0.3

    tau_0 = 'uncalibrated'

    @staticmethod
    def with_planning_scene():
        ManipulatorActions.sc.remove_world_object()
        ManipulatorActions.add_table_plane()
        ManipulatorActions.avoid(*ManipulatorActions.scene_objects)
        time.sleep(0.3)

    @staticmethod
    def without_planning_scene(use_table=True):
        ManipulatorActions.sc.remove_world_object()
        if use_table:
            ManipulatorActions.add_table_plane()
        time.sleep(0.3)


    @staticmethod
    def transforms(q):
        T1 = np.array([[np.cos(q[0]), -np.sin(q[0]), 0, 0],
                      [-np.sin(q[0]), -np.cos(q[0]), 0, 0],
                      [0,0,-1,0.1564],
                      [0,0,0,1]])
        T2 = np.array([[np.cos(q[1]), -np.sin(q[1]), 0, 0],
                      [0, 0, -1, 0.0054],
                      [np.sin(q[1]), np.cos(q[1]), 0, -0.1284],
                      [0, 0, 0, 1]])
        T3 = np.array([[np.cos(q[2]), -np.sin(q[2]), 0, 0],
                      [0, 0, 1, -0.2104],
                      [-np.sin(q[2]), -np.cos(q[2]), 0, -0.0064],
                      [0, 0, 0, 1]])
        T4 = np.array([[np.cos(q[3]), -np.sin(q[3]), 0, 0],
                      [0, 0, -1, -0.0064],
                      [np.sin(q[3]), -np.cos(q[3]), 0, -0.2104],
                      [0, 0, 0, 1]])
        T5 = np.array([[np.cos(q[4]), -np.sin(q[4]), 0, 0],
                      [0, 0, 1, -0.2084],
                      [-np.sin(q[4]), -np.cos(q[4]), 0, -0.0064],
                      [0, 0, 0, 1]])
        T6 = np.array([[np.cos(q[5]), -np.sin(q[5]), 0, 0],
                      [0, 0, -1, 0],
                      [np.sin(q[5]), np.cos(q[5]), 0, -0.1059],
                      [0, 0, 0, 1]])
        T7 = np.array([[np.cos(q[6]), -np.sin(q[6]), 0, 0],
                      [0, 0, 1, -0.1059],
                      [-np.sin(q[6]), -np.cos(q[6]), 0, 0],
                      [0, 0, 0, 1]])
        TE = np.array([[1, 0, 0, 0],
                      [0, -1, 0, 0],
                      [0, 0, -1, -0.0615],
                      [0, 0, 0, 1]])



    @staticmethod
    def goto(pose):
        from kortex_driver.msg import  BaseCyclic_Feedback
        from threading import Lock, Event
        '''
        send the robot to a carteesian goal using the twist command.
        NOT FUNCTIONAL: robot expects translation to be in base_frame and angular velocities in tool frame.
        We expressed here everything in the base_frame. we will need to adjust this to transform the angular velocity to
        the tool_frame (see discussion here: https://github.com/Kinovarobotics/kinova-ros/issues/154). My proposed 
        solution: read the joint angles and write the transformation matrices by hand (numpy) as they are given in the 
        documentation (https://www.kinovarobotics.com/sites/default/files/UG-014_KINOVA_Gen3_Ultra_lightweight_robot-User_guide_EN_R01.pdf 
        page 136). 
        '''

        goal_reached = Event()
        # fb = BaseCyclic_Feedback()
        # goal_pos = np.array([fb.base.tool_pose_x, fb.base.tool_pose_y, fb.base.tool_pose_z])
        goal_pos = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
        goal_rot = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        command_topic = '/my_gen3/in/cartesian_velocity'
        pub = rospy.Publisher(command_topic, TwistCommand, queue_size=1)
        cmd = TwistCommand()

        def control(fb):
            pos = np.array([fb.base.tool_pose_x, fb.base.tool_pose_y, fb.base.tool_pose_z])
            rot = np.array([fb.base.tool_pose_theta_x, fb.base.tool_pose_theta_y, fb.base.tool_pose_theta_z])

            pos_error = goal_pos - pos
            rot_error = goal_rot - rot
            print pos_error
            print rot_error

            # scale both errors such that eef vel will be max 5 cm / s

            vel =  np.linalg.norm(pos_error)
            scale_factor = min(1.0, 0.05 /vel)
            scale_factor = 0.01

            pos_cmd = pos_error * scale_factor
            rot_cmd = rot_error * scale_factor
            cmd.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED
            cmd.duration = 0.0
            cmd.twist.linear_x = pos_cmd[0]
            cmd.twist.linear_y = pos_cmd[1]
            cmd.twist.linear_z = pos_cmd[2]
            cmd.twist.angular_x = rot_cmd[0]
            cmd.twist.angular_y = rot_cmd[1]
            cmd.twist.angular_z = rot_cmd[2]

            pub.publish(cmd)

            if np.linalg.norm(pos_error) < 0.01 and np.linalg.norm(rot_error) < 3:
                # unsub
                ManipulatorActions.fb_sub.unregister()
                # stop motion
                pub.publish(TwistCommand)
                #finish goto thread
                goal_reached.set()

        ManipulatorActions.fb_sub = rospy.Subscriber('/my_gen3/base_feedback', BaseCyclic_Feedback, control, queue_size=1)

        # wait until the robot reaches the final pose
        rospy.sleep(4.0)
        goal_reached.wait()

        return True, 'goto pose'


    @classmethod
    def init_mp(cls):
        movegroup_ns = ManipulatorActions.get_ns()
        robot_description = ManipulatorActions.get_robdesc_ns()
        sc = PlanningSceneInterface(ns=movegroup_ns)
        mg = MoveGroupCommander('arm', ns=movegroup_ns, robot_description=robot_description)
        mg.set_max_acceleration_scaling_factor(0.5)
        mg.set_max_velocity_scaling_factor(0.6)
        mg.set_end_effector_link('tool_frame')
        rospy.sleep(1.0)
        sc.remove_world_object()
        ManipulatorActions.rc = RobotCommander(ns=movegroup_ns)
        ManipulatorActions.mg = mg
        ManipulatorActions.sc = sc
        ManipulatorActions.add_table_plane()

        ManipulatorActions.measure_weight(calibrate=True)
        ManipulatorActions.move_up(home=True)
        ManipulatorActions.measure_weight(calibrate=True)
        return mg, sc

    @classmethod
    def avoid(cls, *objs):
        for obj in objs:
            assert isinstance(obj, myObject)
            # ManipulatorActions.sc.add_box(obj.fname, obj.getPose(pos_offset=[0,0,obj.bb_dim[2]]), tuple(obj.bb_dim))
            pose = PoseStamped()
            pose.header.frame_id = ManipulatorActions.get_move_group().get_planning_frame()
            pose.pose.orientation = obj.orientation
            pose.pose.position = Point(*obj.bb_pos)
            ManipulatorActions.sc.add_box(obj.fname, pose, tuple(obj.bb_dim))
        time.sleep(0.2)

        return True, 'added {} objects to planning scene'.format(len(objs))

    @classmethod
    def add_table_plane(cls):
        table_pose = PoseStamped()
        table_pose.header.frame_id = 'base_link'
        table_pose.pose.orientation.w = 1.0
        ManipulatorActions.sc.add_plane('table', table_pose)

    @classmethod
    def get_move_group(cls):
        # type: () -> MoveGroupCommander
        if isinstance(ManipulatorActions.mg, MoveGroupCommander):
            return ManipulatorActions.mg
        else:
            ManipulatorActions.init_mp()
            return ManipulatorActions.mg

    @classmethod
    def get_planning_scene(cls):
        if isinstance(ManipulatorActions.sc, PlanningSceneInterface):
            return ManipulatorActions.sc
        else:
            ManipulatorActions.init_mp()
            return ManipulatorActions.sc

    @staticmethod
    def get_ns():
        nodes = rosnode.get_node_names()
        for node in nodes:  # type: str
            if 'move_group' in node:
                if 'move_group_commander_wrapper' in node:
                    continue
                split_node = node.split('/')
                i = split_node.index('move_group')
                ns = '/{}'.format(split_node[i-1])
                print(ns)
                return ns
        rospy.logerr('Node move_group node is not running. Exiting.')

    @staticmethod
    def get_robdesc_ns():
        import rosparam
        names = rosparam.get_param_server().getParamNames()
        for name in names: # type: str
            # if name.endswith('robot_description'):
            if name.endswith('robot_description'):
                return name
        # nodes = rosnode.get_node_names()
        # for node in nodes:  # type: str
        #     if 'move_group' in node:
        #         split_node = node.split('/')
        #         i = split_node.index('move_group')
        #         ns = '/{}'.format(split_node[i - 1])
        #         print(ns)
        #         return ns
        # rospy.logerr('Node move_group node is not running. Exiting.')
        #
        # get_robdesc_ns

    @staticmethod
    def isSim():
        if ManipulatorActions.get_ns() == '/':
            return True
        else:
            return False

    @staticmethod
    def get_obj_grasp(obj):
        assert isinstance(obj, myObject)

        grasp_poses = {'glass': ([0.0, 0.03, obj.bb_dim[2] - 0.03], [0,0,0,1]),
                       'mug': ([0.0, -0.03, obj.bb_dim[2] - 0.03], [0,0,0,1]),
                       'knife': ([0.0, 0.00, obj.bb_dim[2] + 0.025], quaternion_from_euler(0,0,np.pi/2.0)),
                       'fork': ([0.0, 0.00, obj.bb_dim[2] + 0.025], quaternion_from_euler(0,0,np.pi/2.0)),
                       'spoon': ([0.0, 0.00, obj.bb_dim[2] + 0.02], quaternion_from_euler(0,0,0)),
                       'scissors': ([0.0, 0.00, obj.bb_dim[2] + 0.01], quaternion_from_euler(0,0,np.pi/2.0))}
        try:
            return grasp_poses[obj.fname]
        except KeyError as e:
            return ([0.0, 0.0, obj.bb_dim[2] - 0.03], [0,0,0,1])

    @staticmethod
    def get_rel_pose(base_pt, base_quat, rel_pt, rel_quat, eef_quat):
        import tf.transformations as trans
        base_pt_mat = trans.translation_matrix(base_pt)
        base_quat_mat = trans.quaternion_matrix(base_quat)
        rel_pt_mat = trans.translation_matrix(rel_pt)
        rel_quat_mat = trans.quaternion_matrix(rel_quat)
        eef_quat_mat = trans.quaternion_matrix(eef_quat)


        # grasp_pose_mat = np.dot(base_pt_mat, np.dot(base_quat_mat, np.dot(rel_pt_mat, rel_quat_mat)))
        grasp_pose_mat = np.dot(base_pt_mat, np.dot(base_quat_mat, np.dot(rel_pt_mat, np.dot(eef_quat_mat, rel_quat_mat))))


        return trans.translation_from_matrix(grasp_pose_mat), ManipulatorActions.normalize_quat(trans.quaternion_from_matrix(grasp_pose_mat))

    @staticmethod
    def normalize_quat(quat):
        quat = np.array(quat)
        return quat / np.sqrt(np.dot(quat, quat))


    @staticmethod
    def getPositioningTopGrasp(obj, gripper_depth=0.0, gripper_span=0.085, make_msg=False):
        assert isinstance(obj, myObject)

        rel_pt, rel_quat = ManipulatorActions.get_obj_grasp(obj)
        eef_rel_quat = ManipulatorActions.getGraspOrientation(0, 0, 0)

        obj_quat = quaternion_from_euler(0.0,0.0,obj.rot)
        # obj_quat = [obj.orientation.x, obj.orientation.y ,obj.orientation.z ,obj.orientation.w]
        point, quat = ManipulatorActions.get_rel_pose(obj.pos, obj_quat, rel_pt, rel_quat, eef_quat=eef_rel_quat)


        # point = np.array(obj.pos) + np.array([0, 0, obj.bb_dim[2] - gripper_depth])
        # quat = ManipulatorActions.getGraspOrientation(0, 0, obj.rot)

        if make_msg:
            pose = PoseStamped()
            pose.header.frame_id = ManipulatorActions.get_move_group().get_planning_frame()
            pose.pose.position = Point(point[0], point[1], point[2])
            pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            return pose
        return [(list(point), quat)]

    @staticmethod
    def getGraspOrientation(x, y, z, return_msg=False):
        eef_rot = quaternion_from_euler(math.pi * x / 180, math.pi + y * math.pi/180, -math.pi / 2 + z * math.pi / 180.0)
        # pose = Pose()
        if return_msg:
            rot = Quaternion()
            rot.x = eef_rot[0]
            rot.y = eef_rot[1]
            rot.z = eef_rot[2]
            rot.w = eef_rot[3]
            return rot
        return eef_rot

    @classmethod
    def move_rel(cls, cmd):
        mg = ManipulatorActions.get_move_group()
        ManipulatorActions.with_planning_scene()
        current_pose = mg.get_current_pose()
        if cmd == "back":
            current_pose.pose.position.x -= 0.1
        if cmd == "forward":
            current_pose.pose.position.x += 0.1
        if cmd == "left":
            current_pose.pose.position.y -= 0.1
        if cmd == "right":
            current_pose.pose.position.y += 0.1

        mg.set_pose_target(current_pose)
        plan = mg.plan()
        if len(plan.joint_trajectory.points) > 1:
            mg.execute(plan)
            return True, ''
        else:
            return False

    @classmethod
    def rotate(cls):
        mg = ManipulatorActions.get_move_group()
        ManipulatorActions.with_planning_scene()
        pose = mg.get_current_pose()
        # pose.pose.position.z = 0.5
        quat = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
        current_euler = euler_from_quaternion(quat)
        pose.pose.orientation = ManipulatorActions.getGraspOrientation(0, 0, (current_euler[2] / math.pi *180 + 180) % 360 , return_msg=True)

        from moveit_msgs.msg import Constraints, PositionConstraint

        eef_on_line = PositionConstraint()
        eef_on_line.header.frame_id = 'base_link'
        eef_on_line.link_name = mg.get_end_effector_link()
        eef_on_line.target_point_offset.x = 0.0
        eef_on_line.target_point_offset.y = 0.0
        eef_on_line.target_point_offset.z = 0.0

        eef_on_line.weight = 1

        tolerance = 0.2
        line = SolidPrimitive()
        line.type = SolidPrimitive.BOX
        line.dimensions.append(0.1)
        line.dimensions.append(0.1)
        line.dimensions.append(0.1)

        eef_on_line.constraint_region.primitives.append(line)
        box_pose = mg.get_current_pose().pose
        box_pose.orientation = Quaternion()
        box_pose.orientation.w = 1
        eef_on_line.constraint_region.primitive_poses.append(box_pose)

        flip_ct = Constraints()
        flip_ct.position_constraints.append(eef_on_line)
        mg.set_path_constraints(flip_ct)

        mg.set_pose_target(pose)
        plan = mg.plan()
        if len(plan.joint_trajectory.points) > 1:
            mg.execute(plan)
            return True, 'rotate successfull'
        else:
            return False, 'rotate not successfull'
        pass

    @classmethod
    def shake(cls):

        ManipulatorActions.with_planning_scene()
        req = SendJointSpeedsCommandRequest()
        req.input.joint_speeds.append(JointSpeed())
        req.input.joint_speeds[-1].joint_identifier = 5
        req.input.joint_speeds[-1].value = -50

        for i in range(6):
            ManipulatorActions.joint_vel_control_srv.call(req)
            req.input.joint_speeds[-1].value *= -1
            rospy.sleep(0.4)


        req.input.joint_speeds[-1].value = 0
        ManipulatorActions.joint_vel_control_srv.call(req)

        # req = SendTwistCommandRequest()
        # req.input.reference_frame = 0
        # req.input.duration = 0.5
        # req.input.twist.linear_z = 1.0
        # req.input.twist.angular_x = 100
        #
        # res = ManipulatorActions.cart_vel_control_srv.call(req)
        #
        # time.sleep(0.3)
        # req.input.twist.linear_z = -1.0
        # req.input.twist.angular_x = -100
        #
        # res = ManipulatorActions.cart_vel_control_srv.call(req)
        #
        # time.sleep(0.3)
        # req.input.twist.linear_z = 0.0
        # req.input.twist.angular_x = 0
        #
        # res = ManipulatorActions.cart_vel_control_srv.call(req)
        # # TODO: call stop srv instead

        return True, 'shaking object'



        pass
        # TODO: implement

    @staticmethod
    def stack(obj1, obj2):
        assert isinstance(obj1, myObject)
        assert isinstance(obj2, myObject)
        ManipulatorActions.without_planning_scene()

        mg = ManipulatorActions.get_move_group()
        current_pose = mg.get_current_pose()
        pose = obj1.getPose()
        pose.pose.position.z = obj1.bb_dim[2] + obj1.bb_dim[2] # TODO: check if we want the gripper depth as well
        pose.pose.orientation = ManipulatorActions.getGraspOrientation(0, 0, obj1.rot, return_msg=True)
        mg.set_pose_target(pose)
        plan = mg.plan()
        if len(plan.joint_trajectory.points) > 1:
            mg.execute(plan)
            return True, 'stacked object {} onto {}'.format(obj1.fname, obj2.fname)
        else:
            return False, 'stacking of object {} onto {} failed'.format(obj1.fname, obj2.fname)

    @classmethod
    def move_above(cls, obj):
        assert isinstance(obj, myObject)
        mg = ManipulatorActions.get_move_group()
        ManipulatorActions.with_planning_scene()
        current_pose = mg.get_current_pose()
        pose = obj.getPose()
        pose.pose.position.z = 0.3
        pose.pose.orientation = ManipulatorActions.getGraspOrientation(0,0,obj.rot, return_msg=True)
        mg.set_pose_target(pose)
        plan = mg.plan()
        if len(plan.joint_trajectory.points) > 1:
            res = mg.execute(plan)
            if res:
                return True, 'move_above {} successful'.format(obj.fname)
            else:
                return False, 'move_above {} unsuccessful. Motion execution failed.'.format(obj.fname)
        else:
            return False, 'move_above {} unsuccessful'.format(obj.fname)

    # @staticmethod
    # def getPositioningTopGrasp(obj, gripper_depth=0.03, gripper_span=0.085):
    #     assert isinstance(obj, myObject)
    #     point = np.array(obj.pos) + np.array([0, 0, obj.bb_dim[2] - gripper_depth])
    #     quat = ManipulatorActions.getOrientation(0, 0, obj.rot)
    #     return [(list(point), quat)]

    @staticmethod
    def getOrientation(x, y, z):
        eef_rot = quaternion_from_euler(0, math.pi, -math.pi / 2 + z * math.pi / 180.0)
        # pose = Pose()
        rot = Quaternion()
        rot.x = eef_rot[0]
        rot.y = eef_rot[1]
        rot.z = eef_rot[2]
        rot.w = eef_rot[3]
        return eef_rot

    @staticmethod
    def goto_cart(wp):
        mg = ManipulatorActions.get_move_group()
        assert isinstance(mg, MoveGroupCommander)
        plan, fraction = mg.compute_cartesian_path(wp, 0.01, 0.)
        raise NotImplementedError

    # TODO: finish or trash

    # @staticmethod
    # def grasp_softly():
    #     # 	ros::Subscriber sub = n.subscribe("/my_gen3/base_feedback",3, feedbackCallback);
    #
    #     def stop_gripper(msg):
    #         assert isinstance(msg, BaseFeedback)
    #         msg.
    #     from kortex_driver.msg import BaseFeedback
    #     rospy.Subscriber('/my_gen3/base_feedback', BaseFeedback, 1, stop_gripper)


    @classmethod
    def approach_grasp(cls, obj, z_offset=0.0):
        assert isinstance(obj, myObject)
        mg = ManipulatorActions.get_move_group()
        ManipulatorActions.without_planning_scene()
        current_pose = mg.get_current_pose()
        # pose = obj.getGraspPose(0.03)
        # pose.pose.position
        pose = ManipulatorActions.getPositioningTopGrasp(obj, gripper_depth=0.02, make_msg=True)
        pose.pose.position.z += z_offset
        # pose.pose.orientation = ManipulatorActions.getGraspOrientation(0, 0, obj.rot)
        pre_pose = copy.deepcopy(pose)
        pre_pose.pose.position.z += 0.05
        mg.set_pose_target(pre_pose)
        plan = mg.plan()
        if len(plan.joint_trajectory.points) > 1:
            mg.execute(plan)
            ManipulatorActions.without_planning_scene(use_table=False)
            rospy.sleep(0.5)
            mg.set_pose_target(pose)
            # ManipulatorActions.sc.remove_world_object('table')
            plan = mg.plan()
            if len(plan.joint_trajectory.points) > 1:
                mg.execute(plan)
                return True, 'approached object {} for grasp'.format(obj.fname)
        else:
            return False, 'motion planning failed'
        # TODO: possibly remove collision body
        # plan = mg.plan()
        # if len(plan.joint_trajectory.points) > 1:
        #     mg.execute(plan)
        #     return True
        # else:
        #     return False


    @classmethod
    def move_up(cls, home=False):
        mg = ManipulatorActions.get_move_group()
        ManipulatorActions.without_planning_scene(use_table=False)
        pose = mg.get_current_pose()
        pose.header.frame_id = mg.get_planning_frame()
        pose.pose.position.z = 0.25
        if home:
            pose.pose.position.x = 0.32
            pose.pose.position.y = 0.0
        mg.set_pose_target(pose)
        plan = mg.plan()
        if len(plan.joint_trajectory.points) > 1:
            mg.execute(plan)
            return True, 'move_up successful'
        else:
            return False, 'move_up not successful'

    @staticmethod
    def send_gripper_command_(value):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION
        # GripperMode.GRIPPER_FORCE

        rospy.loginfo("Sending the gripper command...")

        # Call the service
        try:
            ManipulatorActions.send_gripper_command_srv.call(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False, 'gripper not closed'
        else:
            time.sleep(0.5)
            return True, 'gripper closed'

    @staticmethod
    def send_gripper_command_force(value):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_FORCE
        # GripperMode.GRIPPER_FORCE

        rospy.loginfo("Sending the gripper command...")

        # Call the service
        try:
            ManipulatorActions.send_gripper_command_srv.call(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(0.5)
            return True

    @staticmethod
    def send_gripper_command_speed(value):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_SPEED
        # GripperMode.GRIPPER_FORCE

        rospy.loginfo("Sending the gripper command...")

        # Call the service
        try:
            ManipulatorActions.send_gripper_command_srv.call(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False, 'gripper not closed'
        else:
            time.sleep(0.5)
            return True, 'gripper closed'

    @staticmethod
    def grasp(obj):
        assert isinstance(obj, myObject)
        if ManipulatorActions.isSim():
            from kinova_mujoco.simple_gripper_action_client import send_gripper_command
            send_gripper_command(obj.gripper_setting)
        else:
            ManipulatorActions.send_gripper_command_(obj.gripper_setting)
            return True, 'gripper closed with full force'

    @staticmethod
    def attach(obj):
        print('in attach')
        ManipulatorActions.sc.attach_box(link=ManipulatorActions.get_move_group().get_end_effector_link(),
                                         name=obj.fname,
                                         size=tuple(obj.bb_dim),
                                         touch_links=ManipulatorActions.rc.get_link_names('gripper'))
        return True, ''

    @staticmethod
    def detach():
        ManipulatorActions.sc.remove_attached_object(link=ManipulatorActions.get_move_group().get_end_effector_link())
        return True, 'object detached'

    @staticmethod
    def release():
        if ManipulatorActions.isSim():
            from kinova_mujoco.simple_gripper_action_client import send_gripper_command
            send_gripper_command(pos=0.0)
        else:
            ManipulatorActions.send_gripper_command_(0.0)

        rospy.sleep(1.0)
        ManipulatorActions.detach()
        return True, 'gripper opened, object detached'

    @staticmethod
    def flip_kinova():
        # mg = ManipulatorActions.get_move_group()
        # ManipulatorActions.with_planning_scene()

        req = SendTwistCommandRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED
        req.input.twist.angular_y = 50
        ManipulatorActions.cart_vel_control_srv.call(req)

        rospy.sleep(2.0)
        req.input.twist.angular_y = 0
        ManipulatorActions.cart_vel_control_srv.call(req)

        return True, 'flip successfull'

    @staticmethod
    def measure_weight(calibrate=False, return_m=False):
        mg = ManipulatorActions.get_move_group()
        ManipulatorActions.without_planning_scene()
        res, msg = ManipulatorActions.move_up()
        if not res:
            return False, 'measure weight failed: ' + msg
        ManipulatorActions.with_planning_scene()

        joint_angles = [0.0, 0.0, np.pi, -np.pi*0.5, 0.0, -np.pi*0.5, np.pi]

        plan = mg.plan(joint_angles)

        if len(plan.joint_trajectory.points) == 0:
            return False, 'measure weight unsuccessful'
        mg.execute(plan)

        if not calibrate:
            ManipulatorActions.move_up(home=True)

            plan = mg.plan(joint_angles)

            if len(plan.joint_trajectory.points) == 0:
                return False, 'measure weight unsuccessful'
            mg.execute(plan)

        rospy.sleep(2.0)

        torques = np.zeros(10)
        for i in range(10):
            js = rospy.wait_for_message('/my_gen3/joint_states', JointState)  # type: JointState
            torques[i] = js.effort[3]
            rospy.sleep(0.1)

        if calibrate:
            ManipulatorActions.tau_0 = torques.mean()
            tau_0 = ManipulatorActions.tau_0
        else:
            tau_0 = ManipulatorActions.tau_0

        m = (tau_0 - torques.mean()) / (0.3143 * 9.81)

        if return_m:
            return m
        return True, 'weighing function with motion to measuring pose successfull. m: {}'.format(m)

    @classmethod
    def flip(cls):
        '''
        rotate gripper 180 degree about y
        :return:
        '''

        # return ManipulatorActions.flip_kinova()

        mg = ManipulatorActions.get_move_group()
        ManipulatorActions.with_planning_scene()
        pose = mg.get_current_pose()
        pose.pose.position.z = 0.4
        quat = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
        current_euler = euler_from_quaternion(quat)
        pose.pose.orientation = ManipulatorActions.getGraspOrientation(0, 100, 0, return_msg=True)
        mg.set_pose_target(pose)


        from moveit_msgs.msg import Constraints, PositionConstraint

        eef_on_line = PositionConstraint()
        eef_on_line.header.frame_id = 'base_link'
        eef_on_line.link_name = mg.get_end_effector_link()
        eef_on_line.target_point_offset.x = 0.0
        eef_on_line.target_point_offset.y = 0.0
        eef_on_line.target_point_offset.z = 0.0

        eef_on_line.weight = 1

        tolerance = 0.2
        line = SolidPrimitive()
        line.type = SolidPrimitive.SPHERE
        line.dimensions.append(0.5)
        # line.dimensions.append(2.1)
        # line.dimensions.append(2.1)

        eef_on_line.constraint_region.primitives.append(line)
        box_pose = mg.get_current_pose().pose
        box_pose.orientation = Quaternion()
        box_pose.orientation.w = 1
        eef_on_line.constraint_region.primitive_poses.append(box_pose)
        eef_on_line.header.frame_id = mg.get_planning_frame()

        flip_ct = Constraints()
        flip_ct.position_constraints.append(eef_on_line)
        # mg.set_path_constraints(flip_ct)

        goal_tol = mg.get_goal_position_tolerance()
        goal_tol_rot = mg.get_goal_orientation_tolerance()

        mg.set_goal_position_tolerance(0.3)
        rospy.sleep(0.5)
        mg.set_goal_orientation_tolerance(0.3)
        mg.set_planning_time(10.0)
        rospy.sleep(0.5)

        plan = mg.plan()
        mg.set_goal_position_tolerance(goal_tol)
        mg.set_goal_orientation_tolerance(goal_tol_rot)
        mg.set_planning_time(5.0)


        if len(plan.joint_trajectory.points) > 1:
            mg.execute(plan)
            return True, 'flip successful'
        else:
            return False, 'flip failed'
        pass

    @staticmethod
    def measure_stiffness():
        '''
        rotate gripper 180 degree about y
        :return:
        '''
        return False, 'measure stiffness not implemented'




def test_flip():
    # rospy.init_node('test')
    ManipulatorActions.move_above(scene.objects[2])
    ManipulatorActions.approach_grasp(scene.objects[2])
    ManipulatorActions.grasp(scene.objects[2])
    ManipulatorActions.move_above(scene.objects[2])
    ManipulatorActions.flip()
    ManipulatorActions.move_above(scene.objects[2])
    print('hallo')

def test_rotate():
    # rospy.init_node('test')
    ManipulatorActions.move_above(scene.objects[2])
    ManipulatorActions.approach_grasp(scene.objects[2])
    ManipulatorActions.grasp(scene.objects[2])
    ManipulatorActions.move_above(scene.objects[2])
    ManipulatorActions.rotate()
    ManipulatorActions.move_above(scene.objects[2])
    print('hallo')

def test_move_rel():
    # rospy.init_node('test')
    ManipulatorActions.move_above(scene.objects[2])
    ManipulatorActions.approach_grasp(scene.objects[2])
    ManipulatorActions.grasp(scene.objects[2])
    ManipulatorActions.move_above(scene.objects[2])
    ManipulatorActions.move_rel('back')
    ManipulatorActions.move_rel('left')
    ManipulatorActions.move_rel('forward')
    ManipulatorActions.move_rel('right')

    # ManipulatorActions.move_above(scene.objects[2])
    print('hallo')


def test_goto():
    # mg = ManipulatorActions.get_move_group()
    # current_pose = mg.get_current_pose()
    msg = rospy.wait_for_message('/my_gen3/base_feedback', BaseCyclic_Feedback)  # type: BaseCyclic_Feedback
    current_pose = PoseStamped()
    current_pose.header.frame_id = 'base_frame'
    current_pose.pose.position.x = msg.base.tool_pose_x
    current_pose.pose.position.y = msg.base.tool_pose_y
    current_pose.pose.position.z = msg.base.tool_pose_z

    quat = quaternion_from_euler(np.pi/180*msg.base.tool_pose_theta_x, np.pi/180*msg.base.tool_pose_theta_y,np.pi/180*msg.base.tool_pose_theta_z)
    current_pose.pose.orientation = Quaternion(*quat)

    print('move robot a bit using the gamepad')
    ManipulatorActions.goto(current_pose)

def test_grasp_knife():

    def pick(obj):
        ManipulatorActions.move_above(obj)
        ManipulatorActions.approach_grasp(obj)
        # ManipulatorActions.send_gripper_command_speed(-0.05)
        ManipulatorActions.grasp(obj)
        ManipulatorActions.move_up()

        ManipulatorActions.approach_grasp(obj, z_offset=0.015)
        ManipulatorActions.send_gripper_command_(0.0)
        ManipulatorActions.move_up()

    pick(scene.objects[3])
    # pick(scene.objects[1])
    pick(scene.objects[10])
    pick(scene.objects[7])




    print('hallo')



if __name__=='__main__':
    # just some test code
    from action_classes import V2aScene, ActionSequence, Action

    action_file = 'sample_scenes/questions_with_GT_postproc.json'
    scene_file = 'full_table_scene/scenes_V2A.json'
    folder = os.path.dirname(__file__)

    action_seq = ActionSequence.load_action_sequence('{}/{}'.format(folder, action_file), question_id=6)

    scene_id = action_seq.scene
    scene = V2aScene.load_scene(path='{}/{}'.format(folder, scene_file), scene_id=scene_id)

    print('Scene ID: {}'.format(scene_id))

    import moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('test')

    ManipulatorActions.init_mp()

    name = 'soda_can_full'
    # test_grasp_knife()
    if False:
        weights = np.load('weight_data')
    else:
        weights = np.zeros((1,1,10))

    for obj_idx, obj in enumerate([scene.objects[8]]):
        for grasp in range(1):
            for i in range(10):
                ManipulatorActions.move_above(obj=obj)
                ManipulatorActions.approach_grasp(obj=obj)
                ManipulatorActions.grasp(obj)
                m = ManipulatorActions.measure_weight(return_m=True)
                print(m)
                weights[obj_idx, grasp, i] = m

                ManipulatorActions.move_above(obj=obj)
                ManipulatorActions.approach_grasp(obj=obj, z_offset=0.002)
                ManipulatorActions.send_gripper_command_speed(0.05)
                ManipulatorActions.move_up()

                np.save(name, weights)

    print(weights)
    np.save(name, weights)
    print(ManipulatorActions.tau_0)
    print(weights.mean(), weights.std())
    # print(msg)
    # ManipulatorActions.shake()
    # test_goto()
    # ManipulatorActions.flip()
    # test_flip()
    # test_rotate()
    # test_move_rel()
