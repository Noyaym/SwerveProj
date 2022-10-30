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
    private Utils util;

    public DriveByJoystickCommand(Chassis ch, Utils util) {
        this.chassis = ch;
        this.util = util;
        addRequirements(chassis);
    }

    @Override
    public void execute() {
        double vx = util.getJoystickX(RobotContainer.getJoystickXY());
        double vy = util.getJoystickY(RobotContainer.getJoystickXY());
        double ang = util.getJoystickAngle(RobotContainer.getJoystickDirection());
        SwerveModuleState[] sms = util.getSwerveState(vx, vy, ang);
        chassis.setModules(sms);
    }
    
}
