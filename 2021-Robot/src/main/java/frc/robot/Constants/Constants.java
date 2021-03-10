// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.wpilibj.util.Units;

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
        kSwerveFRSpeed(6, "FrontRightSpeed"), kSwerveFLSpeed(4, "FrontLeftSpeed"), kLifterMotor(8, "LifterMotor"), kHoodMotor(9,"HoodMotor"), kShooter1(10,"Shooter1Motor"),
        kShooter2(11,"Shooter2Motor"),kShooter3(12,"Shooter3Motor");

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

    public static enum CANSparkMaxID {
        agitatorMotor("agitatorMotor",2),intakeMotor("intakeMotor",1);

        public final String name;
        public final int id;

        private CANSparkMaxID(String name, int id){
            this.name = name;
            this.id = id;
        }
    }

    public static enum MotorOutputMultiplier {
        lifter(.3),intake(-1),agitator(-.8),hood(.2);

        public final double multiplier;
        private MotorOutputMultiplier(double multiplier){
            this.multiplier = multiplier;
        }
    }



    public static class DriveConstants{
        // Module 1: Speed 0, Angle 1, Encoder 0 - Back Right
// Module 2: Speed 2, Angle 3, Encoder 1 - Back Left
// Module 3: Speed 4, Angle 5, Encoder 2 - Front Left
// Module 4: Speed 6, Angle 7, Encoder 3 - Front Right    
//0.0122631851627                          
public final static SwerveModuleConstants frontRightConstants = new SwerveModuleConstants

(7, 6, 3, 
.12,  0.0126346014719 , 0.45, 0.004, // ANGLE MOTOR
 0.2, 0.0181327809871 * 2, 0.005, 0.00); // SPEED MOTOR

public final static SwerveModuleConstants backRightConstants = new SwerveModuleConstants
(1, 0, 0,
.12,0.0137420579420 , 0.45, 0.004
, 
 0.2, 0.0210286008586 * 2, 0.005, 0.00);

public final static SwerveModuleConstants frontLeftConstants = new SwerveModuleConstants
(5, 4, 2,
.12, 0.0126346014719 , 0.45, 0.004,
 0.2,0.0181327809871 * 2, 0.005, 0.00);

public final static SwerveModuleConstants backLeftConstants = new SwerveModuleConstants
(3, 2, 1, 
.12, 0.0137420579420 , 0.45, 0.004,
 0.2, 0.0210286008586 * 2, 0.005, 0.0);


}

public static final double shooter_P = .744;
public static final double shooter_D = 0;
public static final double shooter_kS = .714;
public static final double shooter_kV = 0.0275;
public static final double shooter_kA = 0.016;


public static final int talonEncoderResolution = 2048;
public static final double swerveWheelDiam = Units.inchesToMeters(4);
public static final double swerveWheelCircum = swerveWheelDiam * Math.PI;
public static final double swerveDriveMotorGR = 6.86;
// This could be 6 or 6.2
public static final double heightOfPP = 97;
public static final double heightofPPcoT = 91;
public static final double limelightOffsetDegrees = 24.6;
public static final double heightOfLL = 18;
public static final double swerveWheelCircumMeters = swerveWheelDiam * Math.PI;

}