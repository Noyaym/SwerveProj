package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
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

        module = new Module(false, Constants.ModuleConst.FRONT_LEFT_MOVE_MOTOR_ID,
                Constants.ModuleConst.FRONT_LEFT_TURN_MOTOR_ID,
                Constants.ModuleConst.FRONT_LEFT_CANCODER_ID);

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

        SmartDashboard.putData("set vel",
                new InstantCommand(() -> module.setVel(SmartDashboard.getNumber("target vel", 0)), this)
                        .andThen(new InstantCommand(() -> module.setVel(0), this)));

        SmartDashboard.putData("set angle",
                new InstantCommand(() -> module.setAngle(SmartDashboard.getNumber("target angle", 0)), this)
                        .andThen(new InstantCommand(() -> module.setPower(0), this)));

        SmartDashboard.putData("Calibrate", new InstantCommand(() -> {
            module.calibrate();
            System.out.println("calling calibrate!");
        }, this));

        builder.addDoubleProperty("velocity", this::getVelocity, null);
        builder.addDoubleProperty("angle", this::getAngle, null);
        // builder.addDoubleProperty("Error", module.getSeerMotor()::getClosedLoopError,
        // null);
        // builder.addDoubleProperty("Output",
        // module.getSeerMotor()::getMotorOutputPercent, null);
        // builder.addDoubleProperty("SetPoint",
        // module.getSeerMotor()::getClosedLoopTarget, null);

    }

}
