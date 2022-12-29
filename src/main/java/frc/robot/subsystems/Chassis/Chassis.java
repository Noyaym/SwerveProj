package frc.robot.subsystems.Chassis;

import java.io.ObjectInputStream.GetField;
import java.security.PublicKey;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Drive2;
import frc.robot.commands.DriveByJoystickCommand;
import frc.robot.commands.DriveByXboxController;
import frc.robot.utils.SwerveModule;
import frc.robot.utils.Utils;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {
    private final Field2d field;
    private final SwerveModule[] swerveModules;
    private final SwerveModule front_right, back_right, back_left, front_left;
    private final SwerveDriveOdometry odometry;
    private PigeonIMU gyro;
    
    //private ShuffleboardTab tab = Shuffleboard.getTab("chassis data");
    //private NetworkTableEntry fieldEntry = tab.add("Field", 0).getEntry();

    public Chassis(PigeonIMU gyro) {
        this.gyro = gyro;
        this.field = new Field2d();

        Pose2d zeroPose = new Pose2d(0, 0, getRotation2dGyroPosition());
        odometry = new SwerveDriveOdometry(Constants.Kinematics.SWERVE_KINEMATICS, getRotation2dGyroPosition(), zeroPose);

        swerveModules = new SwerveModule[Constants.NUMBER_OF_WHEELS];

        front_right = new SwerveModule(Constants.Offsets.FRONT_RIGHT_OFFSET, Constants.ModuleConst.FRONT_RIGHT_MOVE_MOTOR_ID,
                Constants.ModuleConst.FRONT_RIGHT_TURN_MOTOR_ID,
                Constants.ModuleConst.FRONT_RIGHT_CANCODER_ID, Constants.ModuleConst.FRONT_RIGHT_SET_INVERT_TYPE);
        back_right = new SwerveModule(Constants.Offsets.BACK_RIGHT_OFFSEST, Constants.ModuleConst.BACK_RIGHT_MOVE_MOTOR_ID,
                Constants.ModuleConst.BACK_RIGHT_TURN_MOTOR_ID,
                Constants.ModuleConst.BACK_RIGHT_CANCODER_ID, Constants.ModuleConst.BACK_RIGHT_SET_INVERT_TYPE);
        front_left = new SwerveModule(Constants.Offsets.FRONT_LEFT_OFFSET,
        Constants.ModuleConst.FRONT_LEFT_MOVE_MOTOR_ID,
        Constants.ModuleConst.FRONT_LEFT_TURN_MOTOR_ID,
        Constants.ModuleConst.FRONT_LEFT_CANCODER_ID, Constants.ModuleConst.FRONT_LEFT_SET_INVERT_TYPE);
        back_left = new SwerveModule(Constants.Offsets.BACK_LEFT_OFFSEST, Constants.ModuleConst.BACK_LEFT_MOVE_MOTOR_ID,
                Constants.ModuleConst.BACK_LEFT_TURN_MOTOR_ID,
                Constants.ModuleConst.BACK_LEFT_CANCODER_ID, Constants.ModuleConst.BACK_LEFT_SET_INVERT_TYPE);

        swerveModules[0] = back_right;
        swerveModules[1] = front_left;
        swerveModules[2] = front_right;
        swerveModules[3] = back_left;
        

        setDefaultCommand(new DriveByXboxController(this));

        SmartDashboard.putData("Field", getField());

        SmartDashboard.putData("reset gyro", new InstantCommand(() ->
        setGyroZero()));



    }

    public SwerveModuleState[] getCurrentModuleStates() {
        SwerveModuleState[] sModuleStates = new SwerveModuleState[Constants.NUMBER_OF_WHEELS];
        for (int i = 0; i < sModuleStates.length; i++) {
            sModuleStates[i] = swerveModules[i].getState();
        }
        
        return sModuleStates;

    }

    public SwerveModuleState[] getModulesOptimizedAutonomous(Pose2d currentPose, Pose2d targetPose) {
        SwerveModuleState[] sModuleStates = Utils.driveTo(currentPose, targetPose);
        SwerveModuleState[] sModuleStatesOptimized = new SwerveModuleState[sModuleStates.length];
        

        for (int i = 0; i < sModuleStates.length; i++) {
            double angle = Utils.normalizeAngle(swerveModules[i].getAngle());

            sModuleStatesOptimized[i] = SwerveModuleState.optimize(sModuleStates[i],
            Rotation2d.fromDegrees(angle));

            Rotation2d angleR2D = sModuleStatesOptimized[i].angle;
            
            Double newAngle = Utils.normalizeAngle(angleR2D.getDegrees());
            sModuleStatesOptimized[i] = new SwerveModuleState(sModuleStatesOptimized[i].
            speedMetersPerSecond, Rotation2d.fromDegrees(newAngle));
            

            System.out.println("i=" + i + " optimize " + sModuleStatesOptimized[i].angle.getDegrees()
             + " before opt=" + sModuleStates[i].angle.getDegrees() + " current angle = " 
             + angle);

            //  System.out.println("i=" + i + " optimize " + sModuleStatesOptimized[i].speedMetersPerSecond
            //  + " before opt=" + sModuleStates[i].speedMetersPerSecond + " current angle = " 
            //  + swerveModules[i].getVel());



        }
        System.out.println("--------");


        return sModuleStatesOptimized;

    }

    public SwerveModuleState[] getModulesOptimize(double vx, double vy, double desiredAngle) {

        SwerveModuleState[] sModuleStates = Utils.getSwerveState(vx, vy, desiredAngle);
        SwerveModuleState[] sModuleStatesOptimized = new SwerveModuleState[sModuleStates.length];
        

        for (int i = 0; i < sModuleStates.length; i++) {
            double angle = Utils.normalizeAngle(swerveModules[i].getAngle());

            sModuleStatesOptimized[i] = SwerveModuleState.optimize(sModuleStates[i],
                Rotation2d.fromDegrees(angle));

            Rotation2d angleR2D = sModuleStatesOptimized[i].angle;
            
            Double newAngle = Utils.normalizeAngle(angleR2D.getDegrees());
            sModuleStatesOptimized[i] = new SwerveModuleState(sModuleStatesOptimized[i].
            speedMetersPerSecond, Rotation2d.fromDegrees(newAngle));
            

            System.out.println("i=" + i + " optimize " + sModuleStatesOptimized[i].angle.getDegrees()
             + " before opt=" + sModuleStates[i].angle.getDegrees() + " current angle = " 
             + angle);

            //  System.out.println("i=" + i + " optimize " + sModuleStatesOptimized[i].speedMetersPerSecond
            //  + " before opt=" + sModuleStates[i].speedMetersPerSecond + " current angle = " 
            //  + swerveModules[i].getVel());



        }
        System.out.println("--------");


        return sModuleStatesOptimized;
    }

    public SwerveModuleState[] getModulesOptimize(SwerveModuleState[] sModuleStates) {
        SwerveModuleState[] sModuleStatesOptimized = new SwerveModuleState[sModuleStates.length];
        for (int i = 0; i < sModuleStates.length; i++) {
            double angle = Utils.normalizeAngle(swerveModules[i].getAngle());

            sModuleStatesOptimized[i] = SwerveModuleState.optimize(sModuleStates[i],
                Rotation2d.fromDegrees(angle));

            Rotation2d angleR2D = sModuleStatesOptimized[i].angle;
            
            Double newAngle = Utils.normalizeAngle(angleR2D.getDegrees());
            sModuleStatesOptimized[i] = new SwerveModuleState(sModuleStatesOptimized[i].
            speedMetersPerSecond, Rotation2d.fromDegrees(newAngle));

            System.out.println("i=" + i + " optimize " + sModuleStatesOptimized[i].angle.getDegrees()
             + " before opt=" + sModuleStates[i].angle.getDegrees() + " current angle = " 
             + angle);

        }


        return sModuleStatesOptimized;
    }

    public SwerveModule[] getThisSwerveModules() {
        return swerveModules;
    }

    public double getGyroPosition() {
        return gyro.getFusedHeading();
    }

    public Rotation2d getRotation2dGyroPosition() {
        return Rotation2d.fromDegrees(getGyroPosition());
    }

    public void setModules(SwerveModuleState[] swerveModulesState) {
        for (int i = 0; i < swerveModulesState.length; i++) {
            swerveModules[i].setAngle(swerveModulesState[i].angle.getDegrees());
            swerveModules[i].setVel(swerveModulesState[i].speedMetersPerSecond);
            
        }
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) { 
        odometry.resetPosition(pose, getRotation2dGyroPosition());
    }

    public void odometryUpdate(SwerveModuleState[] swerveModulesState) {
        odometry.update(getRotation2dGyroPosition(), swerveModulesState);
    }

    public void setField(Pose2d pose) {
        field.setRobotPose(pose);
    }

    public void calibrateAll() {
        for (int i=0; i<swerveModules.length; i++) {
            swerveModules[i].calibrate();
        }

        SmartDashboard.putNumber("offset front left", swerveModules[0].getOffset());
        SmartDashboard.putNumber("offset front right", swerveModules[1].getOffset());
        SmartDashboard.putNumber("offset back left", swerveModules[2].getOffset());
        SmartDashboard.putNumber("offset back right", swerveModules[3].getOffset());
    }

    public Field2d getField() {
        return field;
    }

    public void setNeutralModeAngle(boolean isBrake) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setNeutraleModeSteerMotor(isBrake);
        }
    }

    public void setNeutralModeVelocity(boolean isBrake) {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setNeutraleModeMoveMotor(isBrake);
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

    public void setGyroZero() {
        RobotContainer.gyro.setFusedHeading(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("module 0 angle", swerveModules[0].getAngle());
        SmartDashboard.putNumber("module 0 speed", swerveModules[0].getVel());

        SmartDashboard.putNumber("module 1 angle", swerveModules[1].getAngle());
        SmartDashboard.putNumber("module 1 speed", swerveModules[1].getVel());

        SmartDashboard.putNumber("module 2 angle", swerveModules[2].getAngle());
        SmartDashboard.putNumber("module 2 speed", swerveModules[2].getVel());

        SmartDashboard.putNumber("module 3 angle", swerveModules[3].getAngle());
        SmartDashboard.putNumber("module 3 speed", swerveModules[3].getVel());

        SmartDashboard.putData("Calibrate All", new InstantCommand(() ->
        this.calibrateAll()));

        SwerveModuleState[] sms = getCurrentModuleStates();
        odometryUpdate(sms);
        setField(getPose());
        //fieldEntry.setValue(getField());        

    }

}
