package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Module;

public class ModuleOne extends SubsystemBase {

    private final Module module;
    public ModuleOne() {

        module = new Module(false, Constants.ModuleConst.FRONT_RIGHT_MOVE_MOTOR_ID, Constants.ModuleConst.FRONT_RIGHT_TURN_MOTOR_ID,
        Constants.ModuleConst.FRONT_RIGHT_CANCODER_ID);


        setDefaultCommand(new RunCommand(()-> module.setVel(0), this)
        .alongWith(new RunCommand(()->module.setAngle(0), this)));
    }




    @Override
    public void initSendable(SendableBuilder builder) {
        SmartDashboard.putNumber("target vel", 0);
        SmartDashboard.putNumber("target angle", 0);

        SmartDashboard.putData("set vel", new RunCommand(()->
        module.setVel(SmartDashboard.getNumber("target vel", 0)), this));

        SmartDashboard.putData("set angle", new RunCommand(()->
        module.setAngle(SmartDashboard.getNumber("target angle", 0)), this));


    }
    
}
