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
import frc.robot.SwerveModule;

public class ModuleOne extends SubsystemBase {

    private final SwerveModule module;

    public ModuleOne() {

        module = new SwerveModule(false, Constants.ModuleConst.BACK_RIGHT_MOVE_MOTOR_ID,
                Constants.ModuleConst.BACK_RIGHT_TURN_MOTOR_ID,
                Constants.ModuleConst.BACK_RIGHT_CANCODER_ID);
        module.getMoveMotor().setInverted(Constants.ModuleConst.BACK_RIGHT_SET_INVERT_TYPE);

    }

    public double getVelocity() {
        return module.getVel();
    }

    public double getAngle() {
        return module.getAngle();
    }

    public double getSelectedSensorPosition() {
        return module.getSelectedSensorPosition();
    }

    public double getOffset() {
        return module.getOffset();
    }

    public boolean isReverse() {
        return module.isReversed(getAngle());

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        SmartDashboard.putNumber("target vel", 0);
        SmartDashboard.putNumber("target angle", 0);

        SmartDashboard.putData("set vel",
                new RunCommand(() -> module.setVel(SmartDashboard.getNumber("target vel", 0)), this)
                        .andThen(new InstantCommand(() -> module.setVel(0), this)));

        SmartDashboard.putData("set angle",
                new RunCommand(() -> module.setAngle(SmartDashboard.getNumber("target angle", 0)), this)
                        .andThen(new InstantCommand(() -> module.setPowerAngle(0), this)));

        SmartDashboard.putData("Calibrate", new InstantCommand(() -> {
            module.calibrate();
            System.out.println("calling calibrate!");
        }, this));

        builder.addDoubleProperty("velocity", this::getVelocity, null);
        builder.addDoubleProperty("angle", this::getAngle, null);

        builder.addDoubleProperty("offset", this::getOffset, null);
        builder.addBooleanProperty("is reversed", this::isReverse, null);
        builder.addDoubleProperty("motor position", this::getSelectedSensorPosition, null);
        // builder.addDoubleProperty("Error", module.getSeerMotor()::getClosedLoopError,
        // null);
        // builder.addDoubleProperty("Output",
        // module.getSeerMotor()::getMotorOutputPercent, null);
        // builder.addDoubleProperty("SetPoint",
        // module.getSeerMotor()::getClosedLoopTarget, null);

    }

}
