import os
import shutil
from skrobot.data import pr2_urdfpath, fetch_urdfpath

# bit dirty, but we will probably use only pr2 and fetch, so...

# Create urdf under ~/.skrobot
for robot in ['pr2', 'fetch']:
    urdf_path = os.path.expanduser('~/.skrobot/{}_description'.format(robot))
    if not os.path.exists(urdf_path):
        # Create {robot}.urdf
        print("downloading pr2 model... This takes place only once.")
        if robot == 'pr2':
            pr2_urdfpath()
        elif robot == 'fetch':
            fetch_urdfpath()
        # Create {robot}_sensors.urdf for test
        sensor_urdf_in = '../tests/data/{}_sensors.urdf'.format(robot)
        sensor_urdf_out = urdf_path+'/{}_sensors.urdf'.format(robot)
        print('For test, copy {} to {}'.format(
            sensor_urdf_in, sensor_urdf_out))
        shutil.copy(sensor_urdf_in, sensor_urdf_out)
