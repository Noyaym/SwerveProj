// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Module;

public class Chassis extends SubsystemBase {
  private final Module[] swerveModules;
  private final Module front_right, back_right, front_left, back_left;
  private final PigeonIMU gyro;
  private final SwerveDriveOdometry odometry;
  public Chassis() {
    
    gyro = new PigeonIMU(Constants.ChassiConst.jyro_PORT_NUM);
    odometry = new SwerveDriveOdometry(Constants.kinematics.SWERVE_KINEMATICS, getGyroHeading());
    
    swerveModules = new Module[4];

    front_right = new Module(false, Constants.ModuleConst.FRONT_RIGHT_MOVE_MOTOR_ID, Constants.ModuleConst.FRONT_RIGHT_TURN_MOTOR_ID,
    Constants.ModuleConst.FRONT_RIGHT_CANCODER_ID);
    back_right = new Module(false,Constants.ModuleConst.BACK_RIGHT_MOVE_MOTOR_ID, Constants.ModuleConst.BACK_RIGHT_TURN_MOTOR_ID,
        Constants.ModuleConst.BACK_RIGHT_CANCODER_ID);
    front_left = new Module(false,Constants.ModuleConst.FRONT_LEFT_MOVE_MOTOR_ID, Constants.ModuleConst.FRONT_LEFT_TURN_MOTOR_ID,
        Constants.ModuleConst.FRONT_LEFT_CANCODER_ID);
    back_left = new Module(false ,Constants.ModuleConst.BACK_LEFT_MOVE_MOTOR_ID, Constants.ModuleConst.BACK_LEFT_TURN_MOTOR_ID,
        Constants.ModuleConst.BACK_LEFT_CANCODER_ID);

    swerveModules[0] = front_right;
    swerveModules[1] = front_left;
    swerveModules[2] = back_right;
    swerveModules[3] = back_left;

  }

  public void drive(boolean isReversed, double fowardVelocity, double sidewaysVelocity, double angularVelocity) {
    var speeds = Constants.kinematics.SWERVE_KINEMATICS.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(fowardVelocity, sidewaysVelocity, angularVelocity, getGyroHeading()));

    ChassisSpeeds chassisSpeeds = Constants.kinematics.SWERVE_KINEMATICS.toChassisSpeeds(speeds);

    for (int i = 0; i < swerveModules.length; i++) {
      var yoav = SwerveModuleState.optimize(speeds[i], getGyroHeading());
      swerveModules[i].setVel(yoav.speedMetersPerSecond);
      }

      odometryUpdate(speeds);
    }



  public Pose2d odometryUpdate(SwerveModuleState [] SwerveModulesState){
    return odometry.update(getGyroHeading(), SwerveModulesState);
  }

  public Rotation2d getGyroHeading() {
    return Rotation2d.fromDegrees(gyro.getFusedHeading());
  }

  @Override
  public void periodic() {
  }
}
