package frc.robot.commands;

import com.ctre.phoenix.Util;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ChassiConst;
import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.subsystems.Chassis.Utils;

public class DriveByJoystickCommand extends CommandBase {

    private Chassis chassis;

    public DriveByJoystickCommand(Chassis ch) {
        this.chassis = ch;
        addRequirements(chassis);
    }

    @Override
    public void execute() {
        double vx = Utils.getJoystickX(RobotContainer.getJoystickXY());
        double vy = Utils.getJoystickY(RobotContainer.getJoystickXY());
        double ang = Utils.getJoystickAngle(RobotContainer.getJoystickDirection());
        SwerveModuleState[] sms = chassis.getModulesOptimize(vx, vy, ang);
        chassis.setModules(sms);
    }
    
}
