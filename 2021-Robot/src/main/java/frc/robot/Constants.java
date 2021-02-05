// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static enum TalonID {
        // Module 1: Speed 0, Angle 1, Encoder 0 - Back Right
        // Module 2: Speed 2, Angle 3, Encoder 1 - Back Left
        // Module 3: Speed 4, Angle 5, Encoder 2 - Front Left
        // Module 4: Speed 6, Angle 7, Encoder 3 - Front Right
        kSwerveBLAngle(3, "BackLeftAngle"), kSwerveBLSpeed(2, "BackLeftSpeed"), kSwerveBRSpeed(0, "BackRightSpeed"),
        kSwerveBRAngle(1, "BackRightAngle"), kSwerveFRAngle(7, "FrontRightAngle"), kSwerveFLAngle(5, "FrontLeftAngle"),
        kSwerveFRSpeed(6, "FrontRightSpeed"), kSwerveFLSpeed(4, "FrontLeftSpeed");

        public final int id;
        public final String name;

        private TalonID(int id, String name) {
            this.id = id;
            this.name = name;
        }
    }

    public static enum CANDevices {
        kCANCoderBL(1, "CANCoderBackLeft"), kCANCoderBR(0, "CANCoderBackRight"), kCANCoderFL(2, "CANCoderFrontLeft"),
        kCANCoderFR(3, "CANCoderFrontRight");

        public final int id;
        public final String name;

        private CANDevices(int id, String name) {
            this.id = id;
            this.name = name;
        }
    }

    public static final double L = 23.75;
    public static final double W = 24.75;
    public static final int SWERVE_MAX_VOLTS = 0;
    public static final int talonEncoderResolution = 2048;
    public static final int swerveWheelDiam = 4;
    public static final int swerveDriveMotorGR = 0;
    public static final int swerveAngleMotorGR = 0;

    public static enum PIDContants {
        swerveModule("swerveModules",.042,.003,0,0);

        public final String name;
        public final double p;
        public final double i;
        public final double d;
        public final double f;
        
        private PIDContants(String name, double p, double i, double d, double f) {
            this.name = name;
            this.p = p;
            this.i = i;
            this.d = d;
            this.f = f;
        }
    }
}