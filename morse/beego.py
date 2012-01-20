from morse.builder import *
from morse.builder.sensors import *

# Append the robot to the scene
robot = Robot('beego-robot.blend')
robot.name = "beego"

# Append an actuator
motion = Actuator('v_omega')
motion.name = "velocity"
motion.translate(z = 0.3)
robot.append(motion)

# Append a Pose sensor (GPS + Gyroscope)
pose = Sensor('pose')
pose.name = "odometry"
pose.translate(x = -0.12, z = 0.46)
robot.append(pose)

# Append a sick laser
sick = Sensor('sick')
sick.name = "scan"
sick.translate(x = 0.03, z = 0.56)
robot.append(sick)

# Append a camera
camera = Sensor('video_camera')
camera.name = "camera"
camera.translate(x = 0.15, z = 0.582)
robot.append(camera)
camera.properties(cam_width = 128, cam_height = 128, Vertical_Flip = True, capturing = False)

"""
clock = Clock()
clock.name = "clock"
clock.properties(middleware_time = True)
robot.append(clock)
clock.configure_mw('ros')
"""

# Configure the middlewares
motion.configure_mw('ros')
pose.configure_mw('ros')
sick.configure_mw('ros')
camera.configure_mw('ros')

# Select the environement
env = Environment('beego-env.blend')
env.aim_camera([1.0470, 0, 0.7854])
# for virtual machine users:
#env.set_viewport(viewport_shade = 'WIREFRAME')

