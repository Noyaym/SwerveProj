package frc.robot.utils;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Buttons;
import frc.robot.Constants.ChassisConst;
import frc.robot.Constants.Kinematics;

public class Utils {

    private static PIDController mPIDangle2radPerSec = new 
    PIDController(Constants.ChassisConst.ANGLE_2RADPERSEC_Kp, Constants.ChassisConst.ANGLE_2RADPERSEC_Ki,
    Constants.ChassisConst.ANGLE_2RADPERSEC_Kd);

    public static double getJoystickX(Joystick joystick) {
        double x = -joystick.getX();
        double val;
        if (!isJoystickInRange(x)) {
            val = 0.0;
        } else {
            val = normalizeJoystick(x);
        }
        return val;
    }

    public static double getJoystickY(Joystick j) {
        double y = -j.getY();
        double val;
        if (!isJoystickInRange(y)) {
            val = 0.0;
        } else {
            val = normalizeJoystick(y);
        }
        return val;
    }

    public static double getJoystickAngle(Joystick joystick) {
        double y = joystick.getY();
        double x = joystick.getX();
        double angle = Math.atan2(y, x);
        double val;
        if (!isJoystickInRange(angle)) {
            val = 0.0;
        } else {
            val = normalizeJoystick(angle);
        }
        return val;

    }

    public static double normalizeJoystick(double value) {
        return Math.signum(value) * ((1 / 1.1) * Math.pow(value, 2) + (1 - 1 / 1.1));
    }

    /**
     * Checks if joystick value is above a specific value
     * @param value the value
     * @return true if in range, false if not
     */
    public static boolean isJoystickInRange(double value) {
        return Math.abs(value) > Constants.Buttons.JOYSTICK_RANGE;

    }

    public static SwerveModuleState[] getSwerveState(double vx, double vy, double desiredAngle) {
        double dif = desiredAngle - getGyroPosition(RobotContainer.gyro);
        double radPerSec = mPIDangle2radPerSec.calculate(dif);
        Rotation2d currentAngle = Rotation2d.fromDegrees(getGyroPosition(RobotContainer.gyro)); // i am not sure about it but i think this is how its should be done (with the current angle)


        return getModuleStates(vx, vy, radPerSec, currentAngle);
    }

    public static SwerveModuleState[] getModuleStates(double vx, double vy, double radPerSec, Rotation2d currentAngle) {
        ChassisSpeeds cspeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, 0, currentAngle);
        SwerveModuleState[] sModuleStates = Constants.Kinematics.SWERVE_KINEMATICS.toSwerveModuleStates(cspeeds);

        return sModuleStates;
    }

    

    public static double getGyroPosition(PigeonIMU gyro) {
        return gyro.getFusedHeading();
    }

    public Rotation2d getRotation2d(double angle) {
        return new Rotation2d(angle);
    }

}
