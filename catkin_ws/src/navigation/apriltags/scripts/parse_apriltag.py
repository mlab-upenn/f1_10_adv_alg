#with open('/home/thu/catkin_ws/apriltag_dets.txt') as f:
#    line = f.readlines()
#    count = 0
#    pose = []
#    for l in line:
#        if count > 0:
#            dim = (l.rstrip('\n')).split(':')[1].strip()
#            pose.append(float(dim))
#            count = count - 1
#        if count == 0:
#            print pose
#        if 'position:' in l:
#            count = 3
#            pose = []

with open('/home/thu/catkin_ws/src/navigation/apriltags/scripts/apriltag_orientation.txt') as f:
    line = f.readlines()
    for l in line:
        if 'roll' in l:
            rpy = l.rstrip('\n').split('=')[1]
            print rpy
