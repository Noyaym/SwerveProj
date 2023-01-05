package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.utils.Utils;

public class XBoxControl2 extends CommandBase{

    private Chassis chassis;
    private SwerveModuleState[] swerveModulesStates;
    private double vx;
    private double vy;
    private double leftTriggerVal;
    private double rightTriggerVal;


    public XBoxControl2(Chassis ch) {
        this.chassis = ch;
        addRequirements(chassis);
    }

    private void addRequirements(Chassis chassis2) {
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("vx", 0);
        SmartDashboard.putNumber("vy", 0);
        chassis.setNeutralModeAngle(true);
        chassis.setNeutralModeVelocity(true);
    }

    @Override
    public void execute() {
        vx = Utils.getYNormalizedXBox();
        vy = Utils.getXNormalizedXBox();
        leftTriggerVal = Utils.getXBoxControllerTriggerLeft(RobotContainer.xBoxController);
        rightTriggerVal = Utils.getXBoxControllerTriggerRight(RobotContainer.xBoxController);

        if (rightTriggerVal!=0) {
            swerveModulesStates = Utils.getModuleStatesTriggerVal(vx, vy, -rightTriggerVal, 
            Rotation2d.fromDegrees(Utils.getGyroPosition(RobotContainer.gyro)));
        }

        if (leftTriggerVal!=0) {
            swerveModulesStates = Utils.getModuleStatesTriggerVal(vx, vy, leftTriggerVal, 
            Rotation2d.fromDegrees(Utils.getGyroPosition(RobotContainer.gyro)));
        }

        if ((rightTriggerVal!=0) && (leftTriggerVal!=0)) {
            swerveModulesStates = Utils.getModuleStatesTriggerVal(vx, vy, 0, 
            Rotation2d.fromDegrees(Utils.getGyroPosition(RobotContainer.gyro)));
        }
        
        swerveModulesStates = chassis.getModulesOptimize(swerveModulesStates);
        if (vx == 0 && vy == 0 && (rightTriggerVal==0) && (leftTriggerVal==0)) {
            chassis.setPowerAngle(0);
            chassis.setPowerVelocity(0);
        }
        else
            chassis.setModules(swerveModulesStates);
            
    }

    @Override
    public void end(boolean interrupted) {
        chassis.setNeutralModeVelocity(false);
        chassis.setNeutralModeAngle(false);
    }
    
}
