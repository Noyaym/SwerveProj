package frc.robot.subsystems.Chassis;

import java.security.PublicKey;

import com.ctre.phoenix.sensors.PigeonIMU;

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
import frc.robot.Module;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {

    private final Module m1;
    private final Module m2;
    private final Module m3;
    private final Module m4;

   



    public Chassis() {
        m1 = new Module(false, Constants.ModuleConst.CAN_PORT_NUM1,
        Constants.ModuleConst.mVel_PORT_NUM1, Constants.ModuleConst.mAngle_PORT_NUM1);

        m2 = new Module(false, Constants.ModuleConst.CAN_PORT_NUM2,
        Constants.ModuleConst.mVel_PORT_NUM2, Constants.ModuleConst.mAngle_PORT_NUM2);

        m3 = new Module(false, Constants.ModuleConst.CAN_PORT_NUM3,
        Constants.ModuleConst.mVel_PORT_NUM3, Constants.ModuleConst.mAngle_PORT_NUM3);

        m4 = new Module(false, Constants.ModuleConst.CAN_PORT_NUM4,
        Constants.ModuleConst.mVel_PORT_NUM4, Constants.ModuleConst.mAngle_PORT_NUM4);

        


    }

    

    public SwerveModuleState[] getCurrentModuleStates() {
        SwerveModuleState[] sModuleStates = new SwerveModuleState[4];
        sModuleStates[1] = new SwerveModuleState(m1.getVel(), new Rotation2d(m1.getAngle()));
        sModuleStates[2] = new SwerveModuleState(m2.getVel(), new Rotation2d(m2.getAngle()));
        sModuleStates[3] = new SwerveModuleState(m3.getVel(), new Rotation2d(m3.getAngle()));
        sModuleStates[4] = new SwerveModuleState(m4.getVel(), new Rotation2d(m4.getAngle()));
        return sModuleStates;

        
    }




    public double getJyroPosition (PigeonIMU gyro) {
        return gyro.getFusedHeading();
    }

    public Rotation2d getRotation2d(double angle) {
        return new Rotation2d(angle);
    }


    public void setModules (SwerveModuleState[] sms) {
        m1.setAngle(sms[0].angle.getDegrees());
        m1.setVel(sms[0].speedMetersPerSecond);

        m2.setAngle(sms[1].angle.getDegrees());
        m2.setVel(sms[1].speedMetersPerSecond);

        m3.setAngle(sms[2].angle.getDegrees());
        m3.setVel(sms[2].speedMetersPerSecond);

        m4.setAngle(sms[3].angle.getDegrees());
        m4.setVel(sms[3].speedMetersPerSecond);

    }

    public Pose2d getPose2d() { //TO DO:
        return new Pose2d();

    }

    public Field2d getfield2d() { //TO DO:
        return new Field2d();
    }

    


    @Override
    public void periodic() {  
        SwerveModuleState[] sms = getCurrentModuleStates();
        //TO DO: update odometry
        
    }

    
    
    

    



}
