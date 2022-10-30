package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Module;

public class ModuleOne extends SubsystemBase {

    private final Module module;

    public ModuleOne() {
        module = new Module(false, Constants.ModuleConst.CAN_PORT_NUM1,
        Constants.ModuleConst.mVel_PORT_NUM1, Constants.ModuleConst.mAngle_PORT_NUM1);


        
    }

    public double getVelocity() {
        return module.getVel();
    }

    public double getAngle() {
        return module.getAngle();
    }



    @Override
    public void initSendable(SendableBuilder builder) {
        SmartDashboard.putNumber("target vel", 0);
        SmartDashboard.putNumber("target angle", 0);

        SmartDashboard.putData("set vel", new RunCommand(()->
        module.setVel(SmartDashboard.getNumber("target vel", 0)), this)
        .andThen(new InstantCommand(()-> module.setVel(0), this)));

        SmartDashboard.putData("set angle", new RunCommand(()->
        module.setAngle(SmartDashboard.getNumber("target angle", 0)), this)
        .andThen(new InstantCommand(()-> module.setPower(0), this)));

        builder.addDoubleProperty("velocity", this::getVelocity, null);
        builder.addDoubleProperty("angle", this::getAngle, null);
        //builder.addDoubleProperty("Error", module.getSeerMotor()::getClosedLoopError, null);
        //builder.addDoubleProperty("Output", module.getSeerMotor()::getMotorOutputPercent, null);
        //builder.addDoubleProperty("SetPoint", module.getSeerMotor()::getClosedLoopTarget, null);


    }
    
}
