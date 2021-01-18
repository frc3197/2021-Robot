// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain.SwerveDrive;
import frc.robot.subsystems.Drivetrain.SwerveModule;

public class RobotContainer {
  

private static XboxController driver1 = new XboxController(0);


public SwerveModule backLeft = new SwerveModule(Constants.TalonID.kSwerveBLAngle.id, Constants.TalonID.kSwerveBLDrive.id,Constants.CANDevices.kCANCoderBL.id,Constants.SWERVE_MAX_VOLTS);
public SwerveModule backRight = new SwerveModule(Constants.TalonID.kSwerveBRAngle.id, Constants.TalonID.kSwerveBRDrive.id,Constants.CANDevices.kCANCoderBR.id, Constants.SWERVE_MAX_VOLTS); 
public SwerveModule frontLeft = new SwerveModule(Constants.TalonID.kSwerveFLAngle.id, Constants.TalonID.kSwerveFLDrive.id,Constants.CANDevices.kCANCoderFL.id, Constants.SWERVE_MAX_VOLTS);
public SwerveModule frontRight = new SwerveModule(Constants.TalonID.kSwerveFRAngle.id, Constants.TalonID.kSwerveFRDrive.id,Constants.CANDevices.kCANCoderFR.id, Constants.SWERVE_MAX_VOLTS);

public SwerveDrive swerveDrive = new SwerveDrive(backRight, backLeft, frontRight, frontLeft);


  public RobotContainer() {
    swerveDrive.setDefaultCommand(new Drive(swerveDrive));
    configureButtonBindings();
  }

 
  private void configureButtonBindings() {}


  public static double getXLeft(){
    return driver1.getX(Hand.kLeft);
  }

  
  public static double getYLeft(){
    return driver1.getY(Hand.kLeft);
  }

  
  public static double getXRight(){
    return driver1.getX(Hand.kRight);
  }


  /*
  public Command getAutonomousCommand() {
    
    return m_autoCommand;
  }
  */
}
