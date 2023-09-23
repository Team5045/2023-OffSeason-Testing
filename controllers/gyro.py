# time for a pro gyro no lie
import navx
from components.drivetrain import DriveTrain
import wpilib
import ctre
import networktables
from networktables import NetworkTable
from magicbot import MagicRobot

class Gyro():
    # MagicBot
    sd: networktables.NetworkTable
    drivetrain: DriveTrain
    
    def setup(self):
        self.navx = navx.AHRS.create_spi()
        # navx AHRS system

    def balancing(self):
        MAX_RANGE_LIM_HI = 180
        MAX_RANGE_LIM_LOW = -180
        DEFAULT_LOW = -5
        DEFAULT_HI = 5
        angle = self.navx.getRoll()
        print(f'ANGLE_RETURN: {angle}')

        try:
            if (angle < MAX_RANGE_LIM_HI) and (angle > MAX_RANGE_LIM_LOW):
                # Start Checks
                if (angle < DEFAULT_LOW) and (angle > MAX_RANGE_LIM_LOW):
                    self.drivetrain.set_motors(0.33, 0.0)
                    self.sd.putValue("Mode: ", "Moving Forward")
                    print("MODE: ST1")
                    # Forward

                elif (angle > DEFAULT_HI) and (angle < MAX_RANGE_LIM_HI):
                    self.drivetrain.set_motors(-0.33, 0.0)
                    self.sd.putValue("Mode: ", "Moving Backward")
                    print("MODE: ST2")
                    # Backward

                elif (angle < DEFAULT_HI) and (angle > DEFAULT_LOW):
                    self.drivetrain.set_motors(0.0, 0.0)
                    self.sd.putValue("Mode: ", "Balanced!")
                    print("balanced!")
                    print(self.navx.getPitch())
                    # Balanced
            else:
                # Error State
                print("ERROR: OUT OF BOUNDS: E.1")
                self.drivetrain.set_motors(0.0, 0.0)
                self.sd.putValue("Mode: ", "GYRO ERROR")
        except:
            print("ERROR: HARDWARE ERROR")

    def reset(self):
        self.navx.reset()
        self.sd.putValue("MODE: ", "navx_reset")
        print("RESETTING NAVX")
    
    def execute(self):
        pass
            

        
        
        