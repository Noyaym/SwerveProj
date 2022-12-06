package frc.robot.subsystems.Chassis;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Utils {

    private static PIDController PIDangle2radPerSec;

    public Utils() {

        PIDangle2radPerSec = new PIDController(Constants.ChassiConst.a2r_Kp, Constants.ChassiConst.a2r_Ki,
                Constants.ChassiConst.a2r_Kd);
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
        double dif = desiredAngle - getGyroPosition(RobotContainer.gyro);
        double radPerSec = PIDangle2radPerSec.calculate(dif);
        Rotation2d currentAngle = Rotation2d.fromDegrees(getGyroPosition(RobotContainer.gyro)); // i am not sure about it but i think this is how its should be done (with the current angle)


        return getModuleStates(vx, vy, radPerSec, currentAngle);
    }

    public static SwerveModuleState[] getModuleStates(double vx, double vy, double radPerSec, Rotation2d currentAngle) {
        ChassisSpeeds cspeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, radPerSec, currentAngle);
        SwerveModuleState[] sModuleStates = Constants.kinematics.SWERVE_KINEMATICS.toSwerveModuleStates(cspeeds);

        return sModuleStates;
    }

    

    public static double getGyroPosition(PigeonIMU gyro) {
        return gyro.getFusedHeading();
    }

    public Rotation2d getRotation2d(double angle) {
        return new Rotation2d(angle);
    }

}
