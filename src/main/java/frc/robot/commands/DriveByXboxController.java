package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.utils.Utils;

public class DriveByXboxController extends CommandBase{

    private Chassis chassis;

    public DriveByXboxController(Chassis ch) {
        this.chassis = ch;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("vx", 0);
        SmartDashboard.putNumber("vy", 0);
    }

    @Override
    public void execute() {
        double vx = Utils.timesMaxVelocity(Utils.getXboxControllerY(RobotContainer.xBoxController));
        //double vx = SmartDashboard.getNumber("vx", 0);
        double vy = Utils.timesMaxVelocity(Utils.getXboxControllerX(RobotContainer.xBoxController));
        //double vy = SmartDashboard.getNumber("vy", 0);
        boolean isPressed = Utils.isLeftBumperXboxPressed(RobotContainer.xBoxController);

        SwerveModuleState[] sms = Utils.getModuleStates(vx, vy, isPressed, 
        Rotation2d.fromDegrees(Utils.getGyroPosition(RobotContainer.gyro)));

        sms = chassis.getModulesOptimize(sms);

        // SmartDashboard.putNumber("sms angle", sms[1].angle.getDegrees());
        // SmartDashboard.putNumber("sms speed", sms[1].speedMetersPerSecond);
        //&& Utils.getOmega(ang)==0
        
        if (vx == 0 && vy == 0 && !isPressed) {
            chassis.setPowerAngle(0);
            chassis.setPowerVelocity(0);
        }
        else
            chassis.setModules(sms);
    }
    
}
