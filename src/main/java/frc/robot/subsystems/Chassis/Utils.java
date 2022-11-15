package frc.robot.subsystems.Chassis;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Utils {
    // TODO: add kinemati, field, pose
    // private SwerveDriveOdometry odometry;
    private Field2d field;

    public Utils()
    {
        field = new Field2d();
        // odometry = new SwerveDriveOdometry(Constants.kinematics.SWERVE_KINEMATICS, get()); // TODO find a source for
                                                                                           // rotation2d or just use the odometry from chassis
    }

    public static double getJoystickX(Joystick joystick) {
        double x = -joystick.getX();
        double val;
        if ((x < Constants.Buttons.JOYSTICK_RANGE) && (x > -Constants.Buttons.JOYSTICK_RANGE)) {
            val = 0.0;
        } else {
            val = Math.signum(x) * ((1 / 1.1) * Math.pow(x, 2) + (1 - 1 / 1.1));
        }
        return val;
    }

    public static double getJoystickY(Joystick j) {
        double y = -j.getY();
        double val;
        if ((y < Constants.Buttons.JOYSTICK_RANGE) && (y > -Constants.Buttons.JOYSTICK_RANGE)) {
            val = 0.0;
        } else {
            val = Math.signum(y) * ((1 / 1.1) * Math.pow(y, 2) + (1 - 1 / 1.1));
        }
        return val;
    }

    public static double getJoystickAngle(Joystick joystick) {
        double y = joystick.getY();
        double x = joystick.getX();
        double angle = Math.atan(y / x);
        double val;
        if ((angle < Constants.Buttons.JOYSTICK_RANGE) && (angle > -Constants.Buttons.JOYSTICK_RANGE)) {
            val = 0.0;
        } else {
            val = Math.signum(angle) * ((1 / 1.1) * Math.pow(angle, 2) + (1 - 1 / 1.1));
        }
        return val;

    }

    public static SwerveModuleState[] getSwerveState(double vx, double vy, double desiredAngle) {
        // TODO: pid of desired angle to radians per sec

        double radiansPerSec = 0;
        return getModulesOptimaze(vx, vy, radiansPerSec, new Rotation2d());
    }

    public static SwerveModuleState[] getModuleStates(double vx, double vy, double radPerSec, Rotation2d angle) {
        SwerveModuleState[] sModuleStates = new SwerveModuleState[Constants.NUMBER_OF_WHEELS];

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, radPerSec, angle);

        sModuleStates = Constants.kinematics.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);
        return sModuleStates;
    }

    // optimazes the SwerveModuleStates
    public static SwerveModuleState[] getModulesOptimaze(double vx, double vy, double radPerSec, Rotation2d angle) {

        SwerveModuleState[] sModuleStates = getModuleStates(vx, vy, radPerSec, angle);
        SwerveModuleState[] sModuleStatesOptimaze = new SwerveModuleState[sModuleStates.length];

        for (int i = 0; i < sModuleStates.length; i++) {

            sModuleStatesOptimaze[i] = SwerveModuleState.optimize(sModuleStates[i], angle);
        }

        // TODO check
        return sModuleStatesOptimaze;
    }

    public double getJyroPosition(PigeonIMU gyro) {
        return gyro.getFusedHeading();
    }

    public Rotation2d getRotation2d(double angle) {
        return new Rotation2d(angle);
    }

    public Pose2d getPose2d(SwerveModuleState[] SwerveModulesState, Rotation2d currentAngle, SwerveDriveOdometry odometry) {
        return odometry.update(currentAngle, SwerveModulesState);
    }

    public void getField(Pose2d pose){
        field.setRobotPose(pose);
    }

}
