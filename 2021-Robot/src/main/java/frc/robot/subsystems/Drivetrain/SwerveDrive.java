// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {
  // Creates 4 Swerve Modules, each module uses two motors for driving and angle
  public SwerveModule backRight;
  public SwerveModule backLeft;
  public SwerveModule frontRight;
  public SwerveModule frontLeft;

  // L is the WheelBase, distance between the wheels on the y-Axis
  private double L = Constants.L;
  // W is the TrackWidth, distance between the wheels on the x-Axis
  private double W = Constants.W;
  //
  private double gyroHeading;
  public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  // public static AHRS gyro = new AHRS(Port.kUSB);

  public SwerveDrive(SwerveModule backRight, SwerveModule backLeft, SwerveModule frontRight, SwerveModule frontLeft) {
    this.backRight = backRight;
    this.backLeft = backLeft;
    this.frontRight = frontRight;
    this.frontLeft = frontLeft;
  }

  @Override
  public void periodic() {
    // Updates our gyro heading value periodically
    gyroHeading = getHeading();
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  public double getHeading() {
    // The heading is the gyroAngle but when it hits 360 it goes back to zero.
    double heading = getGyroAngle() % 360;
    return heading;
  }

  public void driveFieldCentric(double x1, double y1, double x2) {
    // x1 is the x-axis of the left joystick used for strafing
    // y1 is the y-axis of the left joystick used for driving forward and backward
    // x2 is the x-axis of the right joystick used for rotation

    // This makes it feild centric, look up the math...
    // tbh i dont want to figure out what it does rn so yeah
    double temp = y1 * Math.cos(gyroHeading) + x1 * Math.sin(gyroHeading);

    x1 = -y1 * Math.sin(gyroHeading) + x1 * Math.cos(gyroHeading);

    y1 = temp;

    double r = Math.sqrt((L * L) + (W * W));
    y1 *= -1;
    // Defines A,B,C,D for use in WheelSpeeds and WheelAngle Calculations
    double a = x1 - x2 * (L / r);
    double b = x1 + x2 * (L / r);
    double c = y1 - x2 * (W / r);
    double d = y1 + x2 * (W / r);

    // Calculations for wheel speeds
    double frontRightSpeed = Math.sqrt((b * b) + (c * c));

    double frontLeftSpeed = Math.sqrt((b * b) + (d * d));

    double backLeftSpeed = Math.sqrt((a * a) + (d * d));

    double backRightSpeed = Math.sqrt((a * a) + (c * c));

    // Calculations for wheel angles
    double frontRightAngle = Math.atan2(b, c) / Math.PI;

    double frontLeftAngle = Math.atan2(b, d) / Math.PI;

    double backLeftAngle = Math.atan2(a, d) / Math.PI;

    double backRightAngle = Math.atan2(a, c) / Math.PI;

    /*
     * // This is to normalized wheel speeds to a range of 0 to 1. double max =
     * frontRightSpeed; if(frontLeftSpeed > max ){max = frontLeftSpeed;}
     * if(backLeftSpeed > max ){max = backLeftSpeed;} if(backRightSpeed >
     * max){max=backRightSpeed;} if(max>1){ frontRightSpeed/=max;
     * frontLeftSpeed/=max; backLeftSpeed/=max; backRightSpeed/=max; }
     */

    // Calls the drive function of the 4 modules and feeds in the WheelSpeeds and
    // WheelAngles

    
double max = frontRightSpeed;
if(frontLeftSpeed > max ){max = frontLeftSpeed;}
if(backLeftSpeed > max ){max = backLeftSpeed;} 
if(backRightSpeed > max){max=backRightSpeed;}
if(max>1){
  frontRightSpeed/=max;
  frontLeftSpeed/=max;
  backLeftSpeed/=max;
  backRightSpeed/=max;
} 

    backRight.drive(backRightSpeed, backRightAngle);
    backLeft.drive(backLeftSpeed, backLeftAngle);
    frontRight.drive(frontRightSpeed, frontRightAngle);
    frontLeft.drive(frontLeftSpeed, frontLeftAngle);

  }

  public void turnAllToAngle(double angle) {
    backRight.turnToAngle(angle);
    backLeft.turnToAngle(angle);
    frontLeft.turnToAngle(angle);
    frontRight.turnToAngle(angle);
  }

  public void resetEncoders(){
    backRight.resetEncoder();
    backLeft.resetEncoder();
    frontLeft.resetEncoder();
    frontRight.resetEncoder();
  }

  public void driveRoboCentric(double x1, double y1, double x2) {
    // Same as FeildCentric without the inclusion of the gyro heading to alter
    // inputs

    double r = Math.sqrt((L * L) + (W * W));
    y1 *= -1;

    double a = x1 - x2 * (L / r);
    double b = x1 + x2 * (L / r);
    double c = y1 - x2 * (W / r);
    double d = y1 + x2 * (W / r);

    // Calculations for wheel speeds
    double frontRightSpeed = Math.sqrt((b * b) + (c * c));

    double frontLeftSpeed = Math.sqrt((b * b) + (d * d));

    double backLeftSpeed = Math.sqrt((a * a) + (d * d));

    double backRightSpeed = Math.sqrt((a * a) + (c * c));

    // Calculations for wheel angles
    double frontRightAngle = Math.atan2(b, c) * 180 / Math.PI;

    double frontLeftAngle = Math.atan2(b, d) * 180 / Math.PI;

    double backLeftAngle = Math.atan2(a, d) * 180 / Math.PI;

    double backRightAngle = Math.atan2(a, c) * 180 / Math.PI;
/*
    System.out.println(frontRightAngle + " - frontRightAngle");
    System.out.println(frontRightSpeed + " - frontRightSpeed");

    System.out.println(frontLeftAngle + " - frontLeftAngle");
    System.out.println(frontLeftSpeed + " - frontLeftSpeed");

    System.out.println(backRightAngle + " - backRightAngle");
    System.out.println(backRightSpeed + " - backRightSpeed");

    System.out.println(backLeftAngle + " - backLeftAngle");
    System.out.println(backLeftSpeed + " - backLeftSpeed");

*/
double max = frontRightSpeed;
    if(frontLeftSpeed > max ){max = frontLeftSpeed;}
    if(backLeftSpeed > max ){max = backLeftSpeed;} 
    if(backRightSpeed > max){max=backRightSpeed;}
    if(max>1){
      frontRightSpeed/=max;
      frontLeftSpeed/=max;
      backLeftSpeed/=max;
      backRightSpeed/=max;
    } 

    backRight.drive(backRightSpeed, backRightAngle);
    backLeft.drive(backLeftSpeed, backLeftAngle);
    frontRight.drive(frontRightSpeed, frontRightAngle);
    frontLeft.drive(frontLeftSpeed, frontLeftAngle);
  }
}
