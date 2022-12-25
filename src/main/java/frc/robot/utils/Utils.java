package frc.robot.utils;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Buttons;
import frc.robot.Constants.ChassisConst;
import frc.robot.Constants.Kinematics;

public class Utils {

    private static PIDController pidRad = new 
    PIDController(Constants.ChassisConst.ANGLE_2RADPERSEC_Kp, Constants.ChassisConst.ANGLE_2RADPERSEC_Ki,
    Constants.ChassisConst.ANGLE_2RADPERSEC_Kd);

    private static PIDController pidX = new PIDController(Constants.ChassisConst.errorX_2VX_Kp,
    Constants.ChassisConst.errorX_2VX_Ki, Constants.ChassisConst.errorX_2VX_Kd);
    
    private static PIDController pidY = new PIDController(Constants.ChassisConst.errorY_2VY_Kp,
    Constants.ChassisConst.errorY_2VY_Ki, Constants.ChassisConst.errorY_2VY_Kd);

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
        if (!isJoystickInRange(y)) {
            y = 0.0;}
        double x = joystick.getX();
        if (!isJoystickInRange(x)) {
            x = 0.0; }
        double angle = Math.atan2(y, x);
        return angle*360;

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

    public static double timesMaxVelocity(double valueJoystick) {
        return valueJoystick*Constants.ChassisConst.max_VELOCITY;
    }

    public static double normalizeAngle(double angle) {
        return ((angle%360)+360)%360;
    }

    // public void normalizeModules(SwerveModuleState[] swmBeforeNormalized) {
    //     SwerveModuleState[] aftNormalized = new SwerveModuleState[swmBeforeNormalized.length];
    //     for (int i=0; i<swmBeforeNormalized.length; i++) {
    //         aftNormalized[i] = new SwerveModuleState(swmBeforeNormalized[i].speedMetersPerSecond,
    //         normalizeAngle(swmBeforeNormalized[i].angle));
    //     }
    // }

    public static double optimizeRadDemacia(double difference) {
        if (difference > Math.PI)
            return difference - 2*Math.PI;
        if (difference < -Math.PI) 
            return difference + 2*Math.PI;
        return difference;
    }

    public static double optimizeAngleDemacia(double difference) {
        if (difference > 180)
            return difference - 360;
        if (difference < -180) 
            return difference + 360;
        return difference;
    }

    public static double getOmega(double desiredAngle) {
        double dif = radianFromDegrees(desiredAngle) - radianFromDegrees(getGyroPosition(RobotContainer.gyro));
        dif = optimizeRadDemacia(dif);
        SmartDashboard.putNumber("differenceAngle", dif);
        double radPerSec = pidRad.calculate(dif);
        return radPerSec;
    }

    public static SwerveModuleState[] driveTo(Pose2d currentPose, Pose2d targetPose) {
        double errorX = targetPose.getX() - currentPose.getX();
        double errorY = targetPose.getY() - currentPose.getY();
        double errorRad = targetPose.getRotation().getRadians() - 
        radianFromDegrees(getGyroPosition(RobotContainer.gyro));
        errorRad = optimizeRadDemacia(errorRad);

        double vx = pidX.calculate(errorX);
        double vy = pidY.calculate(errorY);
        double radPerSec = pidRad.calculate(errorRad);
        Rotation2d currentAngle = Rotation2d.fromDegrees(getGyroPosition(RobotContainer.gyro));

        return getModuleStates(vx, vy, radPerSec, currentAngle);

    }

    public static boolean isInPose(Pose2d currentPose, Pose2d targetPose) {
        double errorX = targetPose.getX() - currentPose.getX();
        double errorY = targetPose.getY() - currentPose.getY();
        double errorRad = targetPose.getRotation().getRadians() - 
        radianFromDegrees(getGyroPosition(RobotContainer.gyro));
        errorRad = optimizeRadDemacia(errorRad);

        if (Math.abs(errorX) < Constants.ChassisConst.DEADBAND_AUTONOMOUS 
        && Math.abs(errorY) < Constants.ChassisConst.DEADBAND_AUTONOMOUS && Math.abs(errorRad) 
        < Constants.ChassisConst.DEADBAND_AUTONOMOUS_RAD) {
            return true;
        }

        return false;
    }

    public static boolean isButtoonPressed(JoystickButton button) {
        return button.get();
    }




    public static SwerveModuleState[] getSwerveState(double vx, double vy, double desiredAngle) {
        double dif = radianFromDegrees(desiredAngle) - radianFromDegrees(getGyroPosition(RobotContainer.gyro));
        dif = optimizeRadDemacia(dif);
        SmartDashboard.putNumber("differenceAngle", dif);
        double radPerSec = pidRad.calculate(dif);
        SmartDashboard.putNumber("radPerSec", radPerSec);
        Rotation2d currentAngle = Rotation2d.fromDegrees(getGyroPosition(RobotContainer.gyro)); // i am not sure about it but i think this is how its should be done (with the current angle)


        return getModuleStates(vx, vy, radPerSec, currentAngle);
    }

    public static SwerveModuleState[] getModuleStates(double vx, double vy, double radPerSec, Rotation2d currentAngle) {
        ChassisSpeeds cspeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, 0, currentAngle);
        SwerveModuleState[] sModuleStates = Constants.Kinematics.SWERVE_KINEMATICS.toSwerveModuleStates(cspeeds);

        return sModuleStates;
    }

    public static SwerveModuleState[] getModuleStates(double vx, double vy, boolean isPressed, Rotation2d currentAngle) {
        double rpc = 0;
        if(isPressed) rpc = 1;
        ChassisSpeeds cspeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rpc, currentAngle);
        SwerveModuleState[] sModuleStates = Constants.Kinematics.SWERVE_KINEMATICS.toSwerveModuleStates(cspeeds);

        return sModuleStates;
    }

    

    public static double getGyroPosition(PigeonIMU gyro) {
        return normalizeAngle(gyro.getFusedHeading());
    }

    public static Rotation2d getRotation2d(double angle) {
        return Rotation2d.fromDegrees(angle);
    }

    public static double radianFromDegrees(double degree) {
        degree = normalizeAngle(degree);
        return degree*Math.PI/180;
    }

}
