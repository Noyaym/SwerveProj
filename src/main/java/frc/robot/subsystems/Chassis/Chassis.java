package frc.robot.subsystems.Chassis;

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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {
    private final Field2d field;
    private final SwerveModule[] swerveModules;
    private final SwerveModule front_right, back_right,  back_left; /*front_left,;*/
    private final PigeonIMU gyro;
    private final SwerveDriveOdometry odometry;

    public Chassis() {
        this.field = new Field2d();

        gyro = new PigeonIMU(Constants.ChassiConst.jyro_PORT_NUM);
        odometry = new SwerveDriveOdometry(Constants.kinematics.SWERVE_KINEMATICS, getRotation2d());

        swerveModules = new SwerveModule[Constants.NUMBER_OF_WHEELS];

        front_right = new SwerveModule(false, Constants.ModuleConst.FRONT_RIGHT_MOVE_MOTOR_ID,
                Constants.ModuleConst.FRONT_RIGHT_TURN_MOTOR_ID,
                Constants.ModuleConst.FRONT_RIGHT_CANCODER_ID);
        back_right = new SwerveModule(false, Constants.ModuleConst.BACK_RIGHT_MOVE_MOTOR_ID,
                Constants.ModuleConst.BACK_RIGHT_TURN_MOTOR_ID,
                Constants.ModuleConst.BACK_RIGHT_CANCODER_ID);
        // front_left = new SwerveModule(false, Constants.ModuleConst.FRONT_LEFT_MOVE_MOTOR_ID,
        //         Constants.ModuleConst.FRONT_LEFT_TURN_MOTOR_ID,
        //         Constants.ModuleConst.FRONT_LEFT_CANCODER_ID);
        back_left = new SwerveModule(false, Constants.ModuleConst.BACK_LEFT_MOVE_MOTOR_ID,
                Constants.ModuleConst.BACK_LEFT_TURN_MOTOR_ID,
                Constants.ModuleConst.BACK_LEFT_CANCODER_ID);

        swerveModules[0] = front_right;
        swerveModules[1] = back_left;
        swerveModules[2] = back_right;
        // swerveModules[3] = front_left;

    }

    public SwerveModuleState[] getCurrentModuleStates() { // utills
        SwerveModuleState[] sModuleStates = new SwerveModuleState[4];
        // TO DO:
        return sModuleStates;

    }

    public double getJyroPosition() {
        return gyro.getFusedHeading();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getJyroPosition());
    }

    public void setModules(SwerveModuleState[] sms) { //utils

    }

    


    @Override
    public void periodic() {
        
        SwerveModuleState[] sms = getCurrentModuleStates();
        // TO DO: update odometry

    }

}
