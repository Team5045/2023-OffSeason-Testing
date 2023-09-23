import wpilib
from ctre import WPI_TalonFX
from ctre import WPI_TalonSRX
from magicbot import MagicRobot
from networktables import NetworkTables, NetworkTable
from wpilib import DoubleSolenoid

from components.drivetrain import DriveTrain
from components.boom import Boom
from controllers.gyro import Gyro
import wpilib.drive
from robotpy_ext.autonomous import AutonomousModeSelector
from ctre import NeutralMode
import navx
import rev 
from wpimath.controller import PIDController

# Download and install stuff on the RoboRIO after imaging
'''
py -3 -m robotpy_installer download-python
py -3 -m robotpy_installer install-python
py -3 -m robotpy_installer download robotpy
py -3 -m robotpy_installer install robotpy
py -3 -m robotpy_installer download robotpy[ctre]
py -3 -m robotpy_installer install robotpy[ctre]
py -3 -m robotpy_installer download robotpy[rev]
py -3 -m robotpy_installer install robotpy[rev]
py -3 -m robotpy_installer download pynetworktables
py -3 -m robotpy_installer install pynetworktables
py -3 -m pip install -U robotpy[ctre]
py -3 -m pip install robotpy[ctre]
'''

# Push code to RoboRIO (only after imaging)
'''
python robot/robot.py deploy --skip-tests
py robot/robot.py deploy --skip-tests --no-version-check
'''


INPUT_SENSITIVITY = 0.05

PNEUMATICS_MODULE_TYPE = wpilib.PneumaticsModuleType.CTREPCM
MagicRobot.control_loop_wait_time = 0.05

SPEED_MULTIPLIER = 1
ANGLE_MULTIPLIER = 1
WRIST_ROT_LIMIT = 2000

WINDING_SPEED = .5
BRAKE_MODE = NeutralMode(2)
COAST_MODE = NeutralMode(1)

PID_TARGET_INPUT_MULTIPLIER = 1000

class SpartaBot(MagicRobot):

    # a DriveTrain instance is automatically created by MagicRobot

    drivetrain: DriveTrain
    gyro: Gyro
    boom_arm: Boom
    

    def createObjects(self):
        MOTOR_BRUSHLESS = rev._rev.CANSparkMaxLowLevel.MotorType.kBrushless
        '''Create motors and stuff here'''
        self.boom_extender_motor: rev.CANSparkMax = rev.CANSparkMax(3, MOTOR_BRUSHLESS)
        self.boom_extender_motor_encoder: rev.SparkMaxRelativeEncoder = self.boom_extender_motor.getEncoder()
        NetworkTables.initialize(server='roborio-5045-frc.local')
        self.sd: NetworkTable = NetworkTables.getTable('SmartDashboard')

        self.drive_controller: wpilib.XboxController = wpilib.XboxController(0)  # 0 works for sim?

        self.talon_L_1 = WPI_TalonFX(4) #1 - SQUAREBOT ID #OSBot: 4
        self.talon_L_2 = WPI_TalonFX(8) #5 - SQID #OSBot: 8

        self.talon_R_1 = WPI_TalonFX(7) #6 - SQID #OSBot: 7
        self.talon_R_2 = WPI_TalonFX(6) #9 - SQID #OSBot: 6

        # self.talon_W_1 = WPI_TalonFX(3)
        # self.talon_G_1 = WPI_TalonSRX(1) 
        # self.talon_G_1 = rev.CANSparkMax(2, MOTOR_BRUSHLESS)
        # self.armPID = PIDController(0.00001, 0.0001, 0.0001, 0.02)
        # self.armPID.setTolerance(50)
        # self.arm_pid_target = self.boom_extender_motor_encoder.getPosition()
        # self.arm_pid_output = 0
        # self.arm_pid_enabled = False

        self.boom_rotator_motor1 = WPI_TalonFX(5)
        self.boom_rotator_motor2 = WPI_TalonFX(3)

    def disabledPeriodic(self):
        self.sd.putValue("Mode", "Disabled")

    def teleopInit(self):
        self.sd.putValue("Mode", "Teleop")
        self.boom_extender_motor_encoder.setPosition(0)
        '''Called when teleop starts; optional'''

    def teleopPeriodic(self):
        '''
        Called on each iteration of the control loop\n
        NOTE: all components' execute() methods will be called automatically
        '''
        self.talon_L_1.setNeutralMode(COAST_MODE)
        self.talon_L_2.setNeutralMode(COAST_MODE)
        self.talon_R_1.setNeutralMode(COAST_MODE)
        self.talon_R_2.setNeutralMode(COAST_MODE)
        angle = self.drive_controller.getRightX()
        speed = self.drive_controller.getLeftY()

        intakespeed = 0.2
        STOP_MOTOR = False

        '''ARM'''
        # if (abs(self.drive_controller.getRightTriggerAxis()) > INPUT_SENSITIVITY or abs(
        #         self.drive_controller.getLeftTriggerAxis()) > INPUT_SENSITIVITY):
        #     self.arm_pid_target += self.drive_controller.getRightTriggerAxis() * PID_TARGET_INPUT_MULTIPLIER
        #     self.arm_pid_target -= self.drive_controller.getLeftTriggerAxis() * PID_TARGET_INPUT_MULTIPLIER
        #     self.armPID.setSetpoint(self.arm_pid_target)
        #     if not self.arm_pid_enabled:
        #         self.arm_pid_target = (self.boom_rotator_motor1.getSelectedSensorPosition() + self.boom_rotator_motor2.getSelectedSensorPosition()) / 2
        #         self.armPID.setSetpoint(self.arm_pid_target)
        #         self.arm_pid_enabled = True
        #         self.armPID.reset()

        # if self.drive_controller.getBackButtonReleased():
        #     self.arm_pid_enabled = not self.arm_pid_enabled
        #     if self.arm_pid_enabled:
        #         self.armPID.setSetpoint(self.arm_pid_target)

        # if self.arm_pid_enabled:
        #     self.arm_pid_output = self.armPID.calculate((self.boom_rotator_motor1.getSelectedSensorPosition() + self.boom_rotator_motor2.getSelectedSensorPosition()) / 2)
        #     self.boom_arm.set_rotator(self.arm_pid_output)
        # else:
        #     self.boom_arm.set_rotator(0)
        #     self.armPID.reset()
        
        '''GRABBER'''
        # Releasing the A Button will undo the kill_switch command
        # if(self.drive_controller.getYButton()):
        #     self.talon_G_1.set(intakespeed) # increase/decrease grabber speeds
        #     self.sd.putValue('Grabber: ', 'moving')

        # elif(self.drive_controller.getBButton()):
        #     self.talon_G_1.set(-intakespeed)
        #     self.sd.putValue('Grabber: ', 'moving')

        # elif(self.drive_controller.getAButtonPressed()): 
        #     self.talon_G_1.set(0.0) # kill switch
        #     STOP_MOTOR = True
        #     self.sd.putValue('Grabber_Status: ', STOP_MOTOR)

        # elif(STOP_MOTOR == False):
        #     self.talon_G_1.set(0.04) # default hold-in speeds
        #     self.sd.putValue('Grabber: ', 'holding')

        '''DRIVETRAIN'''
        if (abs(angle) > INPUT_SENSITIVITY or abs(speed) > INPUT_SENSITIVITY):
            self.drivetrain.set_motors(speed, -angle)
            self.sd.putValue('Drivetrain: ', 'moving')
        else:
            # reset value to make robot stop moving
            self.drivetrain.set_motors(0.0, 0.0)
            self.sd.putValue('Drivetrain: ', 'static')

        # wind_speed = 0

        # if (self.drive_controller.getRightBumper()):
        #     wind_speed -= WINDING_SPEED

        # if (self.drive_controller.getLeftBumper()):
        #     wind_speed += WINDING_SPEED

        # self.boom_arm.set_extender(wind_speed, self.boom_extender_motor_encoder)

        # if self.drive_controller.getXButton(): self.gyro.balancing()
        # else:
        #     self.drivetrain.set_motors(0.0, 0.0)
        #     self.talon_L_1.setNeutralMode(BRAKE_MODE)
        #     self.talon_L_2.setNeutralMode(BRAKE_MODE)
        #     self.talon_R_1.setNeutralMode(BRAKE_MODE)
        #     self.talon_R_2.setNeutralMode(BRAKE_MODE)

        # self.sd.putValue("rotator 1 encoder", self.boom_rotator_motor1.getSelectedSensorPosition())
        # self.sd.putValue("rotator 2 encoder", self.boom_rotator_motor2.getSelectedSensorPosition())
        # self.sd.putValue("average rotator encoder", (
        #         self.boom_rotator_motor1.getSelectedSensorPosition() + self.boom_rotator_motor2.getSelectedSensorPosition()) / 2)
        # self.sd.putValue("rotator pid error", self.armPID.getPositionError())
        # self.sd.putValue("rotator pid target", self.arm_pid_target)
        # self.sd.putValue("rotator pid", self.arm_pid_output)
        # self.sd.putValue("pid enabled", self.arm_pid_enabled)

        #NOTE: NAVX on the squarebot is INOP
        

if __name__ == '__main__':
    wpilib.run(SpartaBot)