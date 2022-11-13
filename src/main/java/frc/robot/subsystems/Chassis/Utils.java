package frc.robot.subsystems.Chassis;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Utils {
//TODO: add kinematics, odometry, field, pose

public Utils() {
    //TODO: build kinematics, odometry, field, pose
}



    public static double getJoystickX (Joystick joystick) {
        double x = -joystick.getX();
        double val;
        if ((x<Constants.Buttons.JOYSTICK_RANGE) && (x>-Constants.Buttons.JOYSTICK_RANGE)) {
            val=0.0;
        }
        else {
            val = Math.signum(x)*((1/1.1)*Math.pow(x, 2)+(1-1/1.1));
        }
        return val;
    }

    public static double getJoystickY (Joystick j) {
        double y = -j.getY();
        double val;
        if ((y<Constants.Buttons.JOYSTICK_RANGE) && (y>-Constants.Buttons.JOYSTICK_RANGE)) {
            val=0.0;
        }
        else {
            val = Math.signum(y)*((1/1.1)*Math.pow(y, 2)+(1-1/1.1));
        }
        return val;
    }

    public double getJoystickAngle (Joystick joystick) {
        double y = joystick.getY();
        double x = joystick.getX();
        double angle  = Math.atan(y/x);
        double val;
        if ((angle<Constants.Buttons.JOYSTICK_RANGE) && (angle>-Constants.Buttons.JOYSTICK_RANGE)) {
            val=0.0;
        }
        else {
            val = Math.signum(angle)*((1/1.1)*Math.pow(angle, 2)+(1-1/1.1));
        }
        return val;

    }


    public static SwerveModuleState[] getSwerveState(double vx, double vy, double desiredAngle) {
        //TO DO: pid of desired angle to radians per sec
        double radiansPerSec = 0;
        return getModuleStates(vx, vy, radiansPerSec, new Rotation2d());
    }


    public static SwerveModuleState[] getModuleStates(double vx, double vy, double radPerSec, Rotation2d angle) {
        SwerveModuleState[] sModuleStates = new SwerveModuleState[4];
        //TO DO: kinematics + odometry
        return sModuleStates;
    }


    public double getJyroPosition (PigeonIMU gyro) {
        return gyro.getFusedHeading();
    }

    public Rotation2d getRotation2d(double angle) {
        return new Rotation2d(angle);
    }
    
}
