// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public static final int NUMBER_OF_WHEELS = 4;
    public final class ModuleConst {


        public static final int FRONT_LEFT_MOVE_MOTOR_ID = 7;
        public static final int FRONT_LEFT_TURN_MOTOR_ID = 8;
        public static final int FRONT_LEFT_CANCODER_ID = 11;
        public static final boolean FRONT_LEFT_SET_INVERT_TYPE = true;


        public static final int FRONT_RIGHT_MOVE_MOTOR_ID = 5;
        public static final int FRONT_RIGHT_TURN_MOTOR_ID = 6;
        public static final int FRONT_RIGHT_CANCODER_ID = 13;
        public static final boolean FRONT_RIGHT_SET_INVERT_TYPE = true;

        public static final int BACK_LEFT_MOVE_MOTOR_ID = 1;
        public static final int BACK_LEFT_TURN_MOTOR_ID = 2;
        public static final int BACK_LEFT_CANCODER_ID = 10;

        public static final int BACK_RIGHT_MOVE_MOTOR_ID = 3;
        public static final int BACK_RIGHT_TURN_MOTOR_ID = 4;
        public static final int BACK_RIGHT_CANCODER_ID = 12;
        public static final boolean BACK_RIGHT_SET_INVERT_TYPE = true;

        public static final double mVel_Kp = 0.0058;
        public static final double mVel_Ki = 0.0;
        public static final double mVel_Kd = 0.0006;

        public static final double mAngle_Kp = 0.058;
        public static final double mAngle_Ki = 0;
        public static final double mAngle_Kd = 0.00;

        public static final double PPR_FALCON = 2048;

        public static final double WHEEL_PEREMITER = 0.1016;
        public static final double GEAR_RATIO_VEL = 12.8;
        public static final double PULSE_PER_METER = PPR_FALCON*GEAR_RATIO_VEL/WHEEL_PEREMITER;

        public static final double GEAR_RATIO_ANGLE = 12.8;
        public static final double PULSE_PER_ANGLE = GEAR_RATIO_ANGLE*PPR_FALCON/360;

        public static final double Ks = 0.003;
        public static final double Kv = 0.002;
    }

    public final class Offsets{
        public static final double FRONT_LEFT_OFFSET = 328.53515625;
        public static final double FRONT_RIGHT_OFFSET = 300.673828125;
        public static final double BACK_LEFT_OFFSEST = 47.109375;
        public static final double BACK_RIGHT_OFFSEST = 108.720703125;

    }
    public final class Buttons {
        public static final int JOYSTICK_XY_PORT_NUM = 0;
        public static final int JOYSTICK_DIRECTION_PORT_NUM = 0;
        public static final double JOYSTICK_RANGE = 0;

    }

    public final static class ChassiConst {
        public static final int jyro_PORT_NUM = 14;
        public final static Translation2d[] wheelsMeters = new Translation2d[] {};
        public static final double a2r_Kp = 0;
        public static final double a2r_Ki = 0;
        public static final double a2r_Kd = 0;
    }


    
    public final static class kinematics {
        // Locations for the swerve drive modules relative to the robot center.
        public static final double FRONT_RIGHT_LOCATION_X = 0.262;
        public static final double FRONT_RIGHT_LOCATION_Y = 0.307;
        
        public static final double FRONT_LEFT_LOCATION_X = 0;
        public static final double FRONT_LEFT_LOCATION_Y = 0;
        
        public static final double BACK_LEFT_LOCATION_X = -0.262;
        public static final double BACK_LEFT_LOCATION_Y = -0.307;
        
        public static final double BACK_RIGHT_LOCATION_X = 0.262;
        public static final double BACK_RIGHT_LOCATION_Y = -0.307;
        
        // define modules location
        public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(FRONT_LEFT_LOCATION_X,
        FRONT_LEFT_LOCATION_Y);
        public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(BACK_LEFT_LOCATION_X,
        BACK_LEFT_LOCATION_Y);
        public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(FRONT_RIGHT_LOCATION_X,
        FRONT_RIGHT_LOCATION_Y);
        public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(BACK_RIGHT_LOCATION_X,
        BACK_RIGHT_LOCATION_Y);
        
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            FRONT_LEFT_LOCATION,
            FRONT_RIGHT_LOCATION,
            BACK_LEFT_LOCATION,
            BACK_RIGHT_LOCATION);
        }



        
}
