# coding: UTF-8
import sys
reload(sys)
sys.setdefaultencoding('utf8')
 
import subprocess
import os
import commands
os.system("xterm -e 'bash -c \"cd ~/ddscomm; source devel/setup.bash ; roslaunch rosddsbridge robotdds.launch\"'")

