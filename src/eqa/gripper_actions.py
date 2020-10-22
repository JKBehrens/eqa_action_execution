import copy
import time

import rospy

from kortex_driver.msg import Finger, GripperMode, BaseCyclic_Feedback, RobotiqGripperStatusFlags, MotorFeedback
from kortex_driver.srv import SendGripperCommandRequest, SendGripperCommand, SendGripperCommandResponse, \
    Base_ClearFaults

from action_classes import myObject


class GripperActions(object):

    GRIPPER_SERVICE_TOPIC = '/my_gen3/base/send_gripper_command'
    gripper_command_srv = None

    base_feedback_sub = None

    measurement_times = []
    gripper_states = []  # type: MotorFeedback
    BUFFER_SIZE = 800

    stop_gripper_on_contact = False
    motor_stop_threshold = 0.06

    eval_online = False


    @staticmethod
    def init_gripper_actions():
        GripperActions.gripper_command_srv = rospy.ServiceProxy(GripperActions.GRIPPER_SERVICE_TOPIC,
                                                                SendGripperCommand)
        try:
            GripperActions.gripper_command_srv.wait_for_service(timeout=1.0)
        except rospy.ROSException as e:
            GripperActions.isSim = True
        GripperActions.base_feedback_sub = rospy.Subscriber('/my_gen3/base_feedback', BaseCyclic_Feedback, GripperActions.eval_feedback,
                               queue_size=1)

    @staticmethod
    def calibrate_gripper():
        GripperActions.open_gripper()
        time.sleep(2.0)
        GripperActions.stop_gripper_on_contact = False
        moving = True
        GripperActions.send_gripper_command_speed(-0.05)
        x = [0]
        y = [0.06]
        stalled = 0
        while moving:
            if stalled > 100:
                break
            elif GripperActions.gripper_states[-1].motor[-1].position - x[-1] < 0.2:
                stalled += 1
                time.sleep(0.01)
                continue
            stalled = 0
            y += [GripperActions.gripper_states[-1].motor[-1].current_motor]
            x += [GripperActions.gripper_states[-1].motor[-1].position]
            time.sleep(0.01)


        GripperActions.send_gripper_command_speed(0.0)


        def plot_calib(x,y):
            while not GripperActions.eval_online:
                rospy.sleep(0.1)
            from matplotlib import pyplot as plt
            fig, axs = plt.subplots(1, 1, sharex=True, sharey=False)
            axs.scatter(x,y)
            # axs[1].plot([t.to_sec() for t in GripperActions.measurement_times],
            #             [state.motor[-1].velocity for state in GripperActions.gripper_states])

            fig.savefig('gripper_calib_{}.png'.format(rospy.Time.now().to_sec()))

        plot_calib(x, y)
        print('hallo')

    @staticmethod
    def eval_feedback(msg):
        assert isinstance(msg, BaseCyclic_Feedback)
        # print(msg)
        GripperActions.eval_online = True
        while len(GripperActions.measurement_times) > GripperActions.BUFFER_SIZE:
            GripperActions.measurement_times.pop(0)
            GripperActions.gripper_states.pop(0)
        GripperActions.measurement_times.append(rospy.Time.now())
        GripperActions.gripper_states.append(msg.interconnect.oneof_tool_feedback.gripper_feedback[0])
        motor_current = GripperActions.gripper_states[-1].motor[-1].current_motor

        # motor_speed = gripper_states[-1].motor[-1].velocity

        if GripperActions.stop_gripper_on_contact:
            if motor_current > max(0.06, GripperActions.motor_stop_threshold):
                GripperActions.stop_gripper_on_contact = False
                GripperActions.send_gripper_command_speed(0.0)
                print('stopping gripper motion')
                # print(msg)
                GripperActions.plot_current()

    @staticmethod
    def measure_stiffness(return_val=False):

        GripperActions.open_gripper()
        while not GripperActions.eval_online:
            rospy.sleep(0.1)
        time.sleep(0.3)
        GripperActions.close_gripper_soft(0.06)
        time.sleep(2.0)
        # GripperActions.last_base_feedback.interconnect.oneof_tool_feedback.gripper_feedback[0]
        pos1 = copy.deepcopy(GripperActions.gripper_states[-1].motor[-1].position)

        GripperActions.open_gripper()
        time.sleep(1.0)
        GripperActions.close_gripper_soft(0.4)
        time.sleep(2.0)
        pos2 = copy.deepcopy(GripperActions.gripper_states[-1].motor[-1].position)

        GripperActions.open_gripper()

        if return_val:
            return pos2 - pos1

        if pos2 - pos1 >= 2.0:
            return True, 'object is soft. Deformation: {} % closing'.format(pos2 - pos1)
        else:
            return True,  'object is hard. Deformation: {} % closing'.format(pos2 - pos1)




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
        # req.input.mode = GripperMode.GRIPPER_SPEED

        # GripperMode.GRIPPER_FORCE

        rospy.loginfo("Sending the gripper command...")

        # Call the service
        try:
            GripperActions.gripper_command_srv.call(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            # time.sleep(2.5)
            # plot_current()
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
            GripperActions.gripper_command_srv.call(req)
        except rospy.ServiceException as e:
            print(e)
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            # time.sleep(0.5)
            # time.sleep(2.5)
            # plot_current()
            return True

    @staticmethod
    def grasp(obj):
        assert isinstance(obj, myObject)
        GripperActions.close_gripper_soft(0.06)
        rospy.sleep(0.5)
        while(GripperActions.stop_gripper_on_contact):
            rospy.sleep(0.1)
        return True, 'gripper closed softly'

    @staticmethod
    def close_gripper_soft(threshold):
        GripperActions.stop_gripper_on_contact = True
        threshold = max(0.06, min(threshold, 0.95))  # make sure that threshold has a sensible value
        GripperActions.motor_stop_threshold = threshold
        GripperActions.send_gripper_command_speed(-0.05)
        # send_gripper_command_(0.9)

    @staticmethod
    def open_gripper():
        if GripperActions.send_gripper_command_speed(0.05):
            time.sleep(0.5)
        else:
            return


    @staticmethod
    def plot_current():
        from matplotlib import pyplot as plt
        fig, axs = plt.subplots(2, 1, sharex=True, sharey=False)
        axs[0].plot([t.to_sec() for t in GripperActions.measurement_times], [state.motor[-1].current_motor for state in GripperActions.gripper_states])
        axs[1].plot([t.to_sec() for t in GripperActions.measurement_times], [state.motor[-1].velocity for state in GripperActions.gripper_states])

        fig.savefig('closing_softly_{}.png'.format(rospy.Time.now().to_sec()))

    @staticmethod
    def clear_faults():
        # Init the services
        robot_name = 'my_gen3'
        clear_faults_full_name = '/' + robot_name + '/base/clear_faults'
        rospy.wait_for_service(clear_faults_full_name)
        clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

        clear_faults.call()

        def example_clear_faults():
            try:
                clear_faults()
            except rospy.ServiceException:
                rospy.logerr("Failed to call ClearFaults")
                return False
            else:
                rospy.loginfo("Cleared the faults successfully")
                rospy.sleep(2.5)
                return True

        example_clear_faults()

    @staticmethod
    def gripper_current_closing():
        GripperActions.open_gripper()
        rospy.sleep(1.0)

        while (len(GripperActions.measurement_times) == 0):
            rospy.sleep(1.0)
        t_start = GripperActions.measurement_times[-1]

        GripperActions.send_gripper_command_speed(-0.05)
        rospy.sleep(2.0)
        t_end = GripperActions.measurement_times[-1]
        GripperActions.plot_current()

        print('hello')

def stiffness_exp():
    import numpy as np
    meas = np.zeros(10)
    for i in range(len(meas)):
        deformation = GripperActions.measure_stiffness(return_val=True)
        meas[i] = deformation

    np.save('stiffness_full_can_middle', meas)

    print(meas, meas.mean(), meas.std())
    print('hallo')


if __name__ == '__main__':
    rospy.init_node('gripper_control')


    GripperActions.init_gripper_actions()
    rospy.sleep(1.0)

    GripperActions.calibrate_gripper()

    # GripperActions.gripper_current_closing()




    # GripperActions.gripper_command_srv = rospy.ServiceProxy(GripperActions.GRIPPER_SERVICE_TOPIC, SendGripperCommand)
    # GripperActions.gripper_command_srv.wait_for_service(timeout=1.0)
    # sub = rospy.Subscriber('/my_gen3/base_feedback', BaseCyclic_Feedback, GripperActions.eval_feedback, queue_size=1)
    #
    # GripperActions.close_gripper_soft()
    # rospy.spin()
    # send_gripper_command_speed()
