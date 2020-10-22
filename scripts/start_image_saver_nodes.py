#!/usr/bin/env python
from subprocess import Popen
from eqa import configure_execution as conf
import os
import rospy

print(conf)

my_env = os.environ.copy()

# proc = Popen(['./start_in_other_dir.sh', os.path.expanduser(conf.image_folder), 'pwd'], env=my_env)
# proc.wait()
proc = Popen(['./start_in_other_dir.sh', os.path.expanduser(conf.image_folder), 'roslaunch', 'embodied_question_answering', 'start_image_savers.launch'],
             env=my_env)

while not rospy.is_shutdown():
    rospy.sleep(1.0)

proc.kill()
