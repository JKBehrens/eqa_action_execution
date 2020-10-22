import argparse
import sys
import os
import time

import rospy

from eqa.manipulator_actions import ManipulatorActions
from eqa.gripper_actions import GripperActions

from eqa.action_classes import V2aScene, ActionSequence, Action

import subprocess
import shutil

import eqa.configure_execution as conf


def move_image(source_dir, target_dir):
    # source_dir = '/path/to/source_folder'
    # target_dir = '/path/to/dest_folder'

    abs_path = os.path.expanduser(source_dir)
    file_names = os.listdir(abs_path)

    if not os.path.isdir(target_dir):
        os.mkdir(target_dir)

    images = []
    for file_name in file_names:
        if 'jpg' in file_name:
            shutil.move(os.path.join(abs_path, file_name), target_dir)
            images.append(file_name)

    return images

def take_image(cam='left', image_names=[], image_folder='~/rs_recording', path=''):
    if cam == 'left':
        proc = subprocess.Popen(['rosservice', 'call', '/image_saver1/save'])
    if cam == 'right':
        proc = subprocess.Popen(['rosservice', 'call', '/image_saver2/save'])
    proc.wait()
    images = move_image(image_folder, path + '/images/')
    image_names.append(images)
    return images

def main(q_id):

    folder = os.path.dirname(__file__)
    image_folder = '~/rs_recording'


    # q_id = 260
    # load action sequence from json file
    action_seq = ActionSequence.load_action_sequence('{}/{}'.format(folder,conf.action_file), question_id=q_id)

    # lookup right scene and load it as well
    scene_id = action_seq.scene
    scene = V2aScene.load_scene(path='{}/{}'.format(folder,conf.scene_file), scene_id=scene_id)

    # print scene and action sequence info
    print('Scene ID: {}'.format(scene_id))
    scene.print_objects()
    action_seq.print_seq()

    path = os.path.join(os.path.expanduser(conf.log_base_folder), "Q{}_{}".format(q_id, time.time()))
    try:
        os.mkdir(path)
    except OSError as e:
        print(e)
        print ("Creation of the directory %s failed" % path)
        exit(-11)
    else:
        print ("Successfully created the directory %s " % path)

    # prepare environment, that execution commands will work (init ManipulatorActions, GripperActions, rosnode, load scene, prepare log vars)
    rospy.init_node('action_performer')
    GripperActions.init_gripper_actions()
    objects = scene.objects

    # make the objects available to ManipulatorActions for collision avoidance
    ManipulatorActions.scene_objects = objects

    # log and control vars
    res = True
    log = []
    image_names_left = []
    image_names_right = []

    # loop through action sequence
    for action in action_seq.actions:  #type: Action
        msg = 'no msg specified'

        # if the last action failed, we abort the execution
        if not res:
            break

        # print and say what should happen
        print(action.action_type)
        os.system('spd-say "{}"'.format(action.action_type))

        # take pictures of the scene
        take_image('left', image_names_left, image_folder, path)
        take_image('right', image_names_right, image_folder, path)

        # compile the action (which is usually a single line of pyhton code) to allow also several lines do be executed
        # Every action has to specify as the variables res and msg, where res is the success of the action and msg is a
        # string explaining what happened and also contains the measurement results. This shall be refined later to make
        # it more machine friendly
        cc = compile(action.action_type, 'err.log', mode='exec')

        # execute the action
        exec(cc)

        # append the log message to log
        log.append(msg)

    # take pictures of the final scene
    take_image('left', image_names_left, image_folder, path)
    take_image('right', image_names_right, image_folder, path)

    print log

    # write the log to file in the experiment folder
    with open(path + '/exp_log_q{}_{}.txt'.format(q_id, time.time()), 'w+') as file:
        file.write("Scene: {}".format(scene_id))
        file.write('\n')

        file.write(scene.__repr__())
        file.write('\n')

        file.write(action_seq.__repr__())
        file.write('\n')

        for l in log:
            file.write(l + '\n')
        file.write('\n')


        for i in image_names_left:
            if len(i) == 0:
                continue
            file.write(i[0] + '\n')
        file.write('\n')

        for i in image_names_right:
            if len(i) == 0:
                continue
            file.write(i[0] + '\n')
        file.write('\n')



if __name__ == '__main__':
    # Create the parser
    parser = argparse.ArgumentParser(description='Execute action sequence for a given question.')

    # Add the arguments
    parser.add_argument('Question_ID',
                           metavar='qid',
                           type=int,
                           help='the id of the question')

    # Execute the parse_args() method
    args = parser.parse_args()

    qid = args.Question_ID

    main(q_id=qid)
