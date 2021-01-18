// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static enum TalonID{
        kSwerveBLAngle(0,"BackLeftAngle"),kSwerveBLDrive(1,"BackLeftDrive"),kSwerveBRDrive(2,"BackRightDrive"),kSwerveBRAngle(3,"BackRightAngle"),
        kSwerveFRAngle(4,"FrontRightAngle"),kSwerveFLAngle(5,"FrontLeftAngle"),kSwerveFRDrive(6,"FrontRightDrive"),kSwerveFLDrive(7,"FrontLeftDrive");

        public final int id;
        public final String name;

        private TalonID(int id, String name) {
            this.id = id;
            this.name = name;
          }
    }

    public static enum CANDevices{
        kCANCoderBL(8,"CANCoderBackLeft"),
        kCANCoderBR(9,"CANCoderBackRight"),
        kCANCoderFL(10,"CANCoderFrontLeft"),
        kCANCoderFR(11,"CANCoderFrontRight"),

        public final int id;
        public final String name;

        private CANDevices(int id, String name) {
            this.id = id;
            this.name = name;
          }
    }

	public static final double L = 0;
	public static final double W = 0;
	public static final int SWERVE_MAX_VOLTS = 0;





}
