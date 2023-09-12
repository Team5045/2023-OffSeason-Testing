# Imports for the New Grabber
from components.drivetrain import DriveTrain
import wpilib
import ctre
import networktables
from networktables import NetworkTable
from magicbot import MagicRobot
from ctre import WPI_TalonFX
from ctre import WPI_TalonSRX
from tools.utils import Lim


class grabber:
    # magicbot
    talon_W_1: WPI_TalonFX
    talon_G_1: WPI_TalonSRX
    sd: NetworkTable

    def setup(self):
        self.rotationspeed = 0
        self.suckspeed = 0.15; 
        
    def rotate(self, motor_speed: float):
        self.rotationspeed = Lim.limit(motor_speed, [-0.5, 0.5])
        self.sd.putValue("Wrist Speed: ", motor_speed)

    def grab(self, motor_speed: float):
        self.suckspeed = Lim.limit(motor_speed, [-0.5, 0.5])
        self.sd.putValue("Sucking Speed: ", motor_speed)

    def getPosition(self):
        pos = self.talon_W_1.getSelectedSensorPosition()
        self.sd.putValue("Rotation Pos: ", pos)

    def execute(self):
        self.talon_W_1.set(self.rotationspeed)
        self.talon_G_1.set(self.suckspeed)
