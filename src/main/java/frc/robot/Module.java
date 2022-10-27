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
    private final CANCoder CAN;
    private final SimpleMotorFeedforward ff;

    public Module(boolean isC, int CAN, int vel, int angle) {
        this.mVel = new TalonFX(vel);
        this.mAngle = new TalonFX(angle);
        this.CAN = new CANCoder(CAN);

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
       return CAN.getAbsolutePosition();
    }

    public double getVel() {
        return mVel.getSelectedSensorVelocity()/Constants.ModuleConst.PULSE_PER_METER*10;
    }

    public void setVel(double v) {
        mVel.set(ControlMode.Velocity, v*Constants.ModuleConst.PULSE_PER_METER/10,
        DemandType.ArbitraryFeedForward, ff.calculate(v));
    }

    public void setReversed(double a) {
        if (a-getAngle()>0) {mAngle.setInverted(false);}
        else {mAngle.setInverted(true);}
    }

    public void setAngle(double a) {
        setReversed(a);
        mAngle.set(ControlMode.Position, a*Constants.ModuleConst.PULSE_PER_ANGLE);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Module vel", this::getVel, null);
        builder.addDoubleProperty("Module angle", this::getAngle, null);

        
        
        
    }

    public void calibrate() {
        CAN.setPosition(0);
        mAngle.setSelectedSensorPosition(0);
    }




    
}
