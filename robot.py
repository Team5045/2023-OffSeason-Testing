import wpilib
from ctre import WPI_TalonFX
from ctre import WPI_TalonSRX
from magicbot import MagicRobot
from networktables import NetworkTables, NetworkTable
from wpilib import DoubleSolenoid

from components.drivetrain import DriveTrain
from components.arm import Boom
from components.grabber import grabber
# from controllers.gyro import Gyro
import wpilib.drive
from robotpy_ext.autonomous import AutonomousModeSelector
from ctre import NeutralMode
import navx
import rev 

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

class SpartaBot(MagicRobot):

    # a DriveTrain instance is automatically created by MagicRobot

    drivetrain: DriveTrain
    grabber: grabber
    # gyro: Gyro
    arm: Boom
    

    def createObjects(self):
        MOTOR_BRUSHLESS = rev._rev.CANSparkMaxLowLevel.MotorType.kBrushless
        '''Create motors and stuff here'''
        self.boom_extender_motor: rev.CANSparkMax = rev.CANSparkMax(1, MOTOR_BRUSHLESS)
        self.boom_extender_motor_encoder: rev.SparkMaxRelativeEncoder = self.boom_extender_motor.getEncoder()
        NetworkTables.initialize(server='roborio-5045-frc.local')
        self.sd: NetworkTable = NetworkTables.getTable('SmartDashboard')

        self.drive_controller: wpilib.XboxController = wpilib.XboxController(0)  # 0 works for sim?

        self.talon_L_1 = WPI_TalonFX(4) #1 - SQUAREBOT ID #OSBot: 4
        self.talon_L_2 = WPI_TalonFX(8) #5 - SQID #OSBot: 8

        self.talon_R_1 = WPI_TalonFX(7) #6 - SQID #OSBot: 7
        self.talon_R_2 = WPI_TalonFX(6) #9 - SQID #OSBot: 6

        self.talon_W_1 = WPI_TalonFX(0) # 0 serves as placeholder in this case
        self.talon_G_1 = WPI_TalonSRX(0) # See above comment


    def disabledPeriodic(self):
        self.sd.putValue("Mode", "Disabled")

    def teleopInit(self):
        self.sd.putValue("Mode", "Teleop")
        self.boom_extender_motor_encoder.setPosition(0)
        # self.limelight = NetworkTables.getTable("limelight")
        # self.limelight.LEDState(3)
        # print("limelight on")
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
        # drive controls
        # print("tele")
        angle = self.drive_controller.getRightX()
        speed = self.drive_controller.getLeftY()

        flickspeed = 0
        intakespeed = 0
        
        flickspeed += self.drive_controller.getLeftTriggerAxis()
        flickspeed -= self.drive_controller.getRightTriggerAxis()
        intakespeed += self.drive_controller.getLeftBumper()
        intakespeed -= self.drive_controller.getRightBumper()

        '''WRIST'''
        if(abs(flickspeed) > INPUT_SENSITIVITY):
            self.grabber.rotate(flickspeed)
            if(grabber.getPosition() > WRIST_ROT_LIMIT): self.talon_W_1.set(0.0, 0.0)
            # Setting limit to prevent potential internal damage
        
        '''GRABBER'''
        if(abs(intakespeed) > INPUT_SENSITIVITY):
            self.grabber.grab(intakespeed) # increase/decrease grabber speeds
            self.sd.putValue('Grabber: ', 'moving')
        elif(self.drive_controller.getAButtonPressed()): 
            self.talon_G_1.set(0.0, 0.0) # kill switch
            self.sd.putValue('Grabber: ', 'STOP')
        else: 
            self.talon_G_1.set(0.15, 0.0) # default hold-in speeds
            self.sd.putValue('Grabber: ', 'holding')

        '''DRIVETRAIN'''
        if (abs(angle) > INPUT_SENSITIVITY or abs(speed) > INPUT_SENSITIVITY):
            self.drivetrain.set_motors(speed, -angle)
            self.sd.putValue('Drivetrain: ', 'moving')
        else:
            # reset value to make robot stop moving
            self.drivetrain.set_motors(0.0, 0.0)
            self.sd.putValue('Drivetrain: ', 'static')

        wind_speed = 0

        if (self.drive_controller.getRightBumper()):
            wind_speed -= WINDING_SPEED

        if (self.drive_controller.getLeftBumper()):
            wind_speed += WINDING_SPEED


        # if self.drive_controller.getAButton(): self.gyro.balancing()
        # else:
        #     self.drivetrain.set_motors(0.0, 0.0)
        #     self.talon_L_1.setNeutralMode(BRAKE_MODE)
        #     self.talon_L_2.setNeutralMode(BRAKE_MODE)
        #     self.talon_R_1.setNeutralMode(BRAKE_MODE)
        #     self.talon_R_2.setNeutralMode(BRAKE_MODE)

        #NOTE: NAVX on the squarebot is INOP
        

if __name__ == '__main__':
    wpilib.run(SpartaBot)