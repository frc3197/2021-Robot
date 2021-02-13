// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drive;
import frc.robot.commands.SetVoltage;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.SwerveDrive;
import frc.robot.subsystems.Drivetrain.SwerveModule;

public class RobotContainer {

  private static XboxController driver1 = new XboxController(0);

  
  private JoystickButton driver1A = new JoystickButton(driver1, 1);
  private JoystickButton driver1X = new JoystickButton(driver1, 3);
  private JoystickButton driver1Y = new JoystickButton(driver1, 4);
  private JoystickButton driver1B = new JoystickButton(driver1, 2);
  

  public static SwerveModule backLeft = new SwerveModule(Constants.TalonID.kSwerveBLAngle.id,
      Constants.TalonID.kSwerveBLSpeed.id, Constants.CANDevices.kCANCoderBL.id, Constants.DriveConstants.backLeftConstants);

  public static SwerveModule backRight = new SwerveModule(Constants.TalonID.kSwerveBRAngle.id,
      Constants.TalonID.kSwerveBRSpeed.id, Constants.CANDevices.kCANCoderBR.id, Constants.DriveConstants.backRightConstants);

  public static SwerveModule frontLeft = new SwerveModule(Constants.TalonID.kSwerveFLAngle.id,
      Constants.TalonID.kSwerveFLSpeed.id, Constants.CANDevices.kCANCoderFL.id,Constants.DriveConstants.frontLeftConstants);

  public static SwerveModule frontRight = new SwerveModule(Constants.TalonID.kSwerveFRAngle.id,
      Constants.TalonID.kSwerveFRSpeed.id, Constants.CANDevices.kCANCoderFR.id, Constants.DriveConstants.frontRightConstants);

  public static SwerveDrive swerveDrive = new SwerveDrive(backRight, backLeft, frontRight, frontLeft);

  public static Intake intake = new Intake();

  public static Shooter shooter = new Shooter();

  public RobotContainer() {
    swerveDrive.setDefaultCommand(new Drive(swerveDrive));


    configureButtonBindings();
  }

  private void configureButtonBindings() {
    /*
    driver1A.whileHeld(new SetVoltage(backRight, 6));
    
    driver1X.whileHeld(new SetVoltage(backRight, 7.5));
    
    driver1Y.whileHeld(new SetVoltage(backRight, 7));
    
    driver1B.whileHeld(new SetVoltage(backRight, 6.5));
    */
  }

  public static double getXLeft() {
    double input = driver1.getX(Hand.kLeft);
    if(input < .075 && input > -.075){
      return 0;
    }
    else{
    return input;}
  }

  public static double getYLeft() {
    double input = driver1.getY(Hand.kLeft);
    if(input < .075 && input > -.075){
      return 0;
    }
    else{
    return input;}
  }

  public static double getXRight() {
    double input = driver1.getX(Hand.kRight);
    if(input < .2 && input > -.2){
      return 0;
    }
    else{
    return input;}
  }

  /*
   * public Command getAutonomousCommand() {
   * 
   * return m_autoCommand; }
   */
}
