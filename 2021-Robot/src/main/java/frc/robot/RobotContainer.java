// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveOneMod;
import frc.robot.commands.SwerveToAngle;
import frc.robot.subsystems.Drivetrain.SwerveDrive;
import frc.robot.subsystems.Drivetrain.SwerveModule;

public class RobotContainer {

  private static XboxController driver1 = new XboxController(0);

  private JoystickButton driver1A = new JoystickButton(driver1, 1);
  private JoystickButton driver1X = new JoystickButton(driver1, 2);
  private JoystickButton driver1Y = new JoystickButton(driver1, 4);
  private JoystickButton driver1B = new JoystickButton(driver1, 3);


  public static SwerveModule backLeft = new SwerveModule(Constants.TalonID.kSwerveBLAngle.id,
      Constants.TalonID.kSwerveBLSpeed.id, Constants.CANDevices.kCANCoderBL.id, Constants.SWERVE_MAX_VOLTS);
  public static SwerveModule backRight = new SwerveModule(Constants.TalonID.kSwerveBRAngle.id,
      Constants.TalonID.kSwerveBRSpeed.id, Constants.CANDevices.kCANCoderBR.id, Constants.SWERVE_MAX_VOLTS);
  public static SwerveModule frontLeft = new SwerveModule(Constants.TalonID.kSwerveFLAngle.id,
      Constants.TalonID.kSwerveFLSpeed.id, Constants.CANDevices.kCANCoderFL.id, Constants.SWERVE_MAX_VOLTS);
  public static SwerveModule frontRight = new SwerveModule(Constants.TalonID.kSwerveFRAngle.id,
      Constants.TalonID.kSwerveFRSpeed.id, Constants.CANDevices.kCANCoderFR.id, Constants.SWERVE_MAX_VOLTS);

  public static SwerveDrive swerveDrive = new SwerveDrive(backRight, backLeft, frontRight, frontLeft);

  public RobotContainer() {
    swerveDrive.setDefaultCommand(new Drive(swerveDrive));
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    driver1A.whenPressed(new SwerveToAngle(swerveDrive, 0));
    driver1X.whenPressed(new SwerveToAngle(swerveDrive, 90));
    driver1Y.whenPressed(new SwerveToAngle(swerveDrive, 180));
    driver1B.whenPressed(new SwerveToAngle(swerveDrive, 90));

  }

  public static double getXLeft() {
    return driver1.getX(Hand.kLeft);
  }

  public static double getYLeft() {
    return driver1.getY(Hand.kLeft);
  }

  public static double getXRight() {
    return driver1.getX(Hand.kRight);
  }

  /*
   * public Command getAutonomousCommand() {
   * 
   * return m_autoCommand; }
   */
}
