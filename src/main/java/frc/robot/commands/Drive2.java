package frc.robot.commands;
import com.ctre.phoenix.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ChassisConst;
import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.utils.Utils;

public class Drive2 extends CommandBase{
    Chassis chassis;

    public Drive2(Chassis chassis) {
        this.chassis = chassis;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("vx", 0);
        SmartDashboard.putNumber("vy", 0);
    }

    @Override
    public void execute() {
        double vx = -Utils.timesMaxVelocity(Utils.getJoystickY(RobotContainer.joystickXY));
        //double vx = SmartDashboard.getNumber("vx", 0);
        double vy = Utils.timesMaxVelocity(Utils.getJoystickX(RobotContainer.joystickXY));
        //double vy = SmartDashboard.getNumber("vy", 0);
        boolean isPressed = Utils.isButtoonPressed(RobotContainer.button);

        SwerveModuleState[] sms = Utils.getModuleStatesLeft(vx, vy, isPressed, 
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

