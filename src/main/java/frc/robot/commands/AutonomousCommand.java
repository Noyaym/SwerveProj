package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.utils.Utils;

public class AutonomousCommand extends CommandBase{

    private Chassis chassis;
    private Trajectory trajectory;
    private Pose2d targetPose;

    public AutonomousCommand(Chassis chassis, Path path) throws IOException {
        this.chassis = chassis;
        trajectory = TrajectoryUtil.fromPathweaverJson(path);
        chassis.resetOdometry(trajectory.getInitialPose());
    }

    @Override
    public void initialize() {
        //targetPose =
    }

    @Override
    public void execute() {
        //SwerveModuleState[] swerveModuleStates = Utils.driveTo(chassis.getPose(), targetPose)
    }




    
}
