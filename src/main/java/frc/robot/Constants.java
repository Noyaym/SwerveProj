// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {



    public final class ModuleConst {


        public static final int mVel_PORT_NUM1 = 0;
        public static final int mAngle_PORT_NUM1 = 0;
        public static final int CAN_PORT_NUM1 = 0;

        public static final int mVel_PORT_NUM2 = 0;
        public static final int mAngle_PORT_NUM2 = 0;
        public static final int CAN_PORT_NUM2 = 0;

        public static final int mVel_PORT_NUM3 = 0;
        public static final int mAngle_PORT_NUM3 = 0;
        public static final int CAN_PORT_NUM3 = 0;

        public static final int mVel_PORT_NUM4 = 0;
        public static final int mAngle_PORT_NUM4 = 0;
        public static final int CAN_PORT_NUM4 = 0;

        public static final double mVel_Kp = 0;
        public static final double mVel_Ki = 0;
        public static final double mVel_Kd = 0;

        public static final double mAngle_Kp = 0;
        public static final double mAngle_Ki = 0;
        public static final double mAngle_Kd = 0;


        public static final double PPR_FALCON = 2048;
        public static final double WHEEL_PEREMITER = 0;
        public static final double GEAR_RATIO_VEL = 0;
        public static final double PULSE_PER_METER = PPR_FALCON*GEAR_RATIO_VEL/WHEEL_PEREMITER;

        public static final double GEAR_RATIO_ANGLE = 0;
        public static final double PULSE_PER_ANGLE = GEAR_RATIO_ANGLE*PPR_FALCON/360;

        public static final double Ks = 0;
        public static final double Kv = 0;


    }

    public final class Buttons {
        public static final int joystickxy_PORT_NUM = 0;
        public static final int joystickDirections_PORT_NUM = 0;
        public static final double j_RANGE = 0;

    }

    public final static class ChassiConst {
        public static final int jyro_PORT_NUM = 0;
        public final static Translation2d[] wheelsMeters = new Translation2d[] {};
        public static final double a2r_Kp = 0;
        public static final double a2r_Ki = 0;
        public static final double a2r_Kd = 0;
    }



}
