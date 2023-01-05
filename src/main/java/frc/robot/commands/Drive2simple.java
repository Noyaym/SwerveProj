package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.utils.Utils;

public class Drive2simple extends CommandBase{

    private Chassis chassis;
    private Pose2d targetPose;
    private SwerveModuleState[] swerveModulesStates;

    public Drive2simple(Chassis ch, Pose2d targetPose) {
        this.chassis = ch;
        addRequirements(chassis);
        this.targetPose = targetPose;
    }

    @Override
    public void execute() {
        swerveModulesStates = Utils.driveToSimple(Constants.ChassisConst.AUTONOMOUS_VELOCITY, chassis.getPose(), targetPose);
        chassis.setModules(swerveModulesStates);
        
    }

    @Override
    public boolean isFinished() {
        return Utils.isInPose(chassis.getPose(), targetPose);
    }
    
}
