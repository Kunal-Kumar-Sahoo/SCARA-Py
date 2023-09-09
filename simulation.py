import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF('plane.urdf')
robot = p.loadURDF('scara_bot.urdf')
p.setGravity(0, 0, -10)

for i in range(p.getNumJoints(robot)):
    print(p.getJointInfo(robot, i))

while True:
    target_arm1 = 90
    target_arm2 = 90 

    p.setJointMotorControlArray(robot, [1, 2], p.POSITION_CONTROL, targetPositions=[target_arm1, target_arm2])

    p.stepSimulation()
    time.sleep(1/240)

