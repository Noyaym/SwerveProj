package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Chassis.Utils;

public class ModuleByJoystick extends CommandBase {

    private SwerveModule module;
    private Utils util;
    private Joystick joystick;

    public ModuleByJoystick(SwerveModule module, Utils util) {
        this.module = module;
        this.util = util;
        joystick = RobotContainer.getJoystickXY();
    }

    @Override
    public void execute() {
        double powerSpeed = util.getJoystickX(joystick);
        double powerAngle = util.getJoystickY(joystick);

        module.setPowerVelocity(powerSpeed);
        module.setPowerAngle(powerAngle);
    }

    @Override
    public void end(boolean interrupted) {
        module.setPowerVelocity(0);
        module.setPowerAngle(0);
    }

}
