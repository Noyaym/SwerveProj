package frc.robot.subsystems.Chassis;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.SwerveModule;

public class Utils {
    // TODO: add kinemati, field, pose
    //private PIDController steerPid; //what is this PID used for?
    
    private static PIDController PIDangle2radPerSec;

    public Utils()
    {
        //steerPid.setP(Constants.ModuleConst.mAngle_Kp);
        //steerPid.setI(Constants.ModuleConst.mAngle_Ki);
        //steerPid.setD(Constants.ModuleConst.mAngle_Kd);
        PIDangle2radPerSec = new PIDController(Constants.ChassiConst.a2r_Kp, Constants.ChassiConst.a2r_Ki,
        Constants.ChassiConst.a2r_Kd);
        // odometry = new SwerveDriveOdometry(Constants.kinematics.SWERVE_KINEMATICS, get()); // TODO find a source for
         //what is this?                                                                                  // rotation2d or just use the odometry from chassis
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
        double dif = desiredAngle - getGyroPosition(RobotContainer.getGyro());
        double radPerSec = PIDangle2radPerSec.calculate(dif);

        return getModulesOptimize(vx, vy, radPerSec, new Rotation2d());
    }

    public static SwerveModuleState[] getModuleStates(double vx, double vy, double radPerSec, Rotation2d angle) {
        ChassisSpeeds cspeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, radPerSec, angle);
        SwerveModuleState[] sModuleStates = Constants.kinematics.SWERVE_KINEMATICS.toSwerveModuleStates(cspeeds);
        return sModuleStates;
    }

    // optimazes the SwerveModuleStates
    public static SwerveModuleState[] getModulesOptimize(double vx, double vy, double radPerSec, Rotation2d angle) {

        SwerveModuleState[] sModuleStates = getModuleStates(vx, vy, radPerSec, angle);
        SwerveModuleState[] sModuleStatesOptimaze = new SwerveModuleState[sModuleStates.length];

        for (int i = 0; i < sModuleStates.length; i++) {

            sModuleStatesOptimaze[i] = SwerveModuleState.optimize(sModuleStates[i], angle); //TODO: this is not true. look at the function parameters.
        }

        // TODO check
        //what is check?
        return sModuleStatesOptimaze;
    }

    public static double getGyroPosition(PigeonIMU gyro) {
        return gyro.getFusedHeading();
    }

    public Rotation2d getRotation2d(double angle) {
        return new Rotation2d(angle);
    }
    



}
