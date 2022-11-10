package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class Module implements Sendable {

    private final TalonFX mVel;
    private final TalonFX mAngle;
    private final CANCoder encoder;
    private final SimpleMotorFeedforward ff;
    private double offset;

    public Module(boolean isC, int vel, int angle, int CAN) {
        this.mVel = new TalonFX(vel);
        this.mAngle = new TalonFX(angle);
        this.encoder = new CANCoder(CAN);

        mVel.configFactoryDefault();
        mAngle.configFactoryDefault();

        mVel.config_kP(0, Constants.ModuleConst.mVel_Kp);
        mVel.config_kI(0, Constants.ModuleConst.mVel_Ki);
        mVel.config_kD(0, Constants.ModuleConst.mVel_Kd);

        mAngle.config_kP(0, Constants.ModuleConst.mAngle_Kp);
        mAngle.config_kI(0, Constants.ModuleConst.mAngle_Ki);
        mAngle.config_kD(0, Constants.ModuleConst.mAngle_Kd);

        this.ff = new SimpleMotorFeedforward(Constants.ModuleConst.Ks, Constants.ModuleConst.Kv);

        if (isC) {
            calibrate();
        }

    }

    public double getAngle() {
        return encoder.getAbsolutePosition()-offset;
    }

    public double getVel() {
        return mVel.getSelectedSensorVelocity() / Constants.ModuleConst.PULSE_PER_METER * 10;
    }

    public double getOffset() {
        return offset;
    }

    public void setVel(double velocity) {
        mVel.set(ControlMode.Velocity, velocity * Constants.ModuleConst.PULSE_PER_METER / 10,
                DemandType.ArbitraryFeedForward, ff.calculate(velocity));
    }

    public void setReversed(double angle) {
        if (angle - getAngle() > 0) {
            mAngle.setInverted(true);
        } else {
            mAngle.setInverted(false);
        }
    }

    public boolean isReversed(double angle) {
        if (angle - getAngle() > 0) {
            return true; }
        return false;

    }

    public double convertAngle2Pulse(double angle) {
        return angle *Constants.ModuleConst.PULSE_PER_ANGLE;
    }

    public double convertOffset2Pulse() {
        return offset*Constants.ModuleConst.PULSE_PER_ANGLE;
    }

    public double calcFF(double angle) {
        return convertAngle2Pulse(10); //still not sure about that
    }

    public void setAngle(double angle) {
        //setReversed(angle);
        mAngle.set(ControlMode.Position, convertAngle2Pulse(angle),
        DemandType.ArbitraryFeedForward, calcFF(angle));
        SmartDashboard.putNumber("angle error", mAngle.getClosedLoopError());
    }
    

    public void setPower(double power) {
        mAngle.set(ControlMode.PercentOutput, power);
    }

    public TalonFX getMoveMotor() {
        return mVel;
    }

    public TalonFX getSteerMotor() {
        return mAngle;
    }

    public double getSelectedSensorPosition() {
        return mAngle.getSelectedSensorPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Module vel", this::getVel, null);
        builder.addDoubleProperty("Module angle", this::getAngle, null);

    }

    public void calibrate() {
        offset = encoder.getAbsolutePosition();
        mAngle.setSelectedSensorPosition(0);
    }

}
