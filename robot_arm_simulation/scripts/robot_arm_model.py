import pybullet as p
import numpy as np

class Robot_Arm_Model:

  def __init__(self, urdfRootPath=''):
    self.urdfRootPath = urdfRootPath
    self.joint_names = ["base_link_upper_arm", "lower_arm_gripper_arm", "upper_arm_lower_arm"]  
    #TODO attention: check the sensor msg here, make sure joint_names have the same order as in the message. 
    self.reset()
      
  def buildJointNameToIdDict(self):
    nJoints = p.getNumJoints(self.robot_arm)
    self.jointNameToId = {}
    for i in range(nJoints):
      jointInfo = p.getJointInfo(self.robot_arm, i)
      self.jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
    self.resetPose()
    for i in range(100):
      p.stepSimulation()

  def buildMotorIdList(self):
    for joint_name in self.joint_names:   
      self.motorIdList.append(self.jointNameToId[joint_name])

  def reset(self):
    self.robot_arm = p.loadURDF("%s/arms_xacro.urdf" % self.urdfRootPath, 0, 0, 0)
    self.kp = 0.2
    self.kd = 0.1
    self.maxForce = 10000
    self.nMotors = 3
    self.motorIdList = []
    self.motorDir = [-1, -1, -1]    #multiplier for each joint
    self.buildJointNameToIdDict()
    self.buildMotorIdList()

  def setMotorAngleById(self, motorId, desiredAngle):
    p.setJointMotorControl2(bodyIndex=self.robot_arm,
                            jointIndex=motorId,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=desiredAngle,
                            positionGain=self.kp,
                            velocityGain=self.kd,
                            force=self.maxForce)

  def setMotorAngleByName(self, motorName, desiredAngle):
    self.setMotorAngleById(self.jointNameToId[motorName], desiredAngle)

  def resetPose(self):
    for joint_name in self.joint_names: 
      p.resetJointState(self.robot_arm, self.jointNameToId[joint_name],
                      0.0)
      self.setMotorAngleByName(joint_name, 0.0)

  def getBasePosition(self):
    position, orientation = p.getBasePositionAndOrientation(self.robot_arm)
    return position

  def getBaseOrientation(self):
    position, orientation = p.getBasePositionAndOrientation(self.robot_arm)
    return orientation

  def applyAction(self, motorCommands):
    for i in range(self.nMotors):
      self.setMotorAngleById(self.motorIdList[i], motorCommands[i])

  def getMotorAngles(self):
    motorAngles = []
    for i in range(self.nMotors):
      jointState = p.getJointState(self.robot_arm, self.motorIdList[i])
      motorAngles.append(jointState[0])
    motorAngles = np.multiply(motorAngles, self.motorDir)
    return motorAngles

  def getMotorVelocities(self):
    motorVelocities = []
    for i in range(self.nMotors):
      jointState = p.getJointState(self.robot_arm, self.motorIdList[i])
      motorVelocities.append(jointState[1])
    motorVelocities = np.multiply(motorVelocities, self.motorDir)
    return motorVelocities

  def getMotorTorques(self):
    motorTorques = []
    for i in range(self.nMotors):
      jointState = p.getJointState(self.robot_arm, self.motorIdList[i])
      motorTorques.append(jointState[3])
    motorTorques = np.multiply(motorTorques, self.motorDir)
    return motorTorques
