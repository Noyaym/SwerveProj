package frc.robot.subsystems.Chassis;

import java.io.ObjectInputStream.GetField;
import java.security.PublicKey;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.sensors.PigeonIMU;
import frc.robot.subsystems.Chassis.Utils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {
    private final Field2d field;
    private final SwerveModule[] swerveModules;
    private final SwerveModule front_right, back_right, back_left, front_left;
    private final SwerveDriveOdometry odometry;
    private PigeonIMU gyro;

    public Chassis(PigeonIMU gyro) {
        this.gyro = gyro;
        this.field = new Field2d();

        odometry = new SwerveDriveOdometry(Constants.kinematics.SWERVE_KINEMATICS, getRotation2d());

        swerveModules = new SwerveModule[Constants.NUMBER_OF_WHEELS];

        front_right = new SwerveModule(false, Constants.ModuleConst.FRONT_RIGHT_MOVE_MOTOR_ID,
                Constants.ModuleConst.FRONT_RIGHT_TURN_MOTOR_ID,
                Constants.ModuleConst.FRONT_RIGHT_CANCODER_ID);
        back_right = new SwerveModule(false, Constants.ModuleConst.BACK_RIGHT_MOVE_MOTOR_ID,
                Constants.ModuleConst.BACK_RIGHT_TURN_MOTOR_ID,
                Constants.ModuleConst.BACK_RIGHT_CANCODER_ID);
        front_left = new SwerveModule(false,
                Constants.ModuleConst.FRONT_LEFT_MOVE_MOTOR_ID, 
                Constants.ModuleConst.FRONT_LEFT_TURN_MOTOR_ID,
                Constants.ModuleConst.FRONT_LEFT_CANCODER_ID);
        back_left = new SwerveModule(false, Constants.ModuleConst.BACK_LEFT_MOVE_MOTOR_ID,
                Constants.ModuleConst.BACK_LEFT_TURN_MOTOR_ID,
                Constants.ModuleConst.BACK_LEFT_CANCODER_ID);

        swerveModules[0] = front_right;
        swerveModules[1] = back_left;
        swerveModules[2] = back_right;
        swerveModules[3] = front_left;

        SmartDashboard.putData("Field", getField());

    }

    public SwerveModuleState[] getCurrentModuleStates() {
        SwerveModuleState[] sModuleStates = new SwerveModuleState[Constants.NUMBER_OF_WHEELS];
        for (int i = 0; i < sModuleStates.length; i++) {
            sModuleStates[i] = swerveModules[i].getState();
        }
        
        return sModuleStates;

    }

    public SwerveModule[] getThisSwerveModules() {
        return swerveModules;
    }

    public double getGyroPosition() {
        return gyro.getFusedHeading();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getGyroPosition());
    }

    public void setModules(SwerveModuleState[] SwerveModulesState) {
        for (int i = 0; i < SwerveModulesState.length; i++) {
            // SwerveModulesState[i].speedMetersPerSeconds
        }
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) { 
        odometry.resetPosition(pose, getRotation2d());
    }

    public void odometryUpdate(SwerveModuleState[] SwerveModulesState) {
        odometry.update(getRotation2d(), SwerveModulesState);
    }

    public void setField(Pose2d pose) {
        field.setRobotPose(pose);
    }

    public Field2d getField() {
        return field;
    }

    public void setNeutralModeAngle(boolean isBrake) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setNeutraleModeSteerMotor(isBrake);
        }
    }

    public void setPowerAngle(double power) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setPowerAngle(power);
        }
    }

    public void setAngle(double angle) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setAngle(angle);
        }
    }

    public void setPowerVelocity(double power) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setPowerVelocity(power);
        }
    }

    @Override
    public void periodic() {
        SwerveModuleState[] sms = getCurrentModuleStates();
        odometryUpdate(sms);
        setField(getPose());

    }

}
