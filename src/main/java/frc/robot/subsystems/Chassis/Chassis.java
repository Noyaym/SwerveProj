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

    //TO DO

   



    public Chassis() {
        //TO DO

        


    }

    

    public SwerveModuleState[] getCurrentModuleStates() {
        SwerveModuleState[] sModuleStates = new SwerveModuleState[4];
        //TO DO:
        return sModuleStates;

        
    }




    public double getJyroPosition (PigeonIMU gyro) {
        return gyro.getFusedHeading();
    }

    public Rotation2d getRotation2d(double angle) {
        return new Rotation2d(angle);
    }


    public void setModules (SwerveModuleState[] sms) {
        //TO DO:

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
