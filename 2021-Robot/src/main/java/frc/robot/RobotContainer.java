// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drive;
import frc.robot.commands.SetVoltage;
import frc.robot.commands.beamIncrement;
import frc.robot.commands.forceShoot;
import frc.robot.commands.moveHood;
import frc.robot.commands.runHopper;
import frc.robot.commands.runIntake;
import frc.robot.commands.shoot;
import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.SwerveDrive;
import frc.robot.subsystems.Drivetrain.SwerveModule;

public class RobotContainer {

  private static XboxController driver1 = new XboxController(0);
  private static XboxController driver2 = new XboxController(1);

  private JoystickButton driver1A = new JoystickButton(driver1, 1);
  private JoystickButton driver1X = new JoystickButton(driver1, 3);
  private JoystickButton driver1Y = new JoystickButton(driver1, 4);
  private JoystickButton driver1B = new JoystickButton(driver1, 2);

  private JoystickButton driver2RB = new JoystickButton(driver2, 6);


  public static SwerveModule backLeft = new SwerveModule(Constants.TalonID.kSwerveBLAngle.id,
      Constants.TalonID.kSwerveBLSpeed.id, Constants.CANDevices.kCANCoderBL.id,
      Constants.DriveConstants.backLeftConstants,false,true);

  public static SwerveModule backRight = new SwerveModule(Constants.TalonID.kSwerveBRAngle.id,
      Constants.TalonID.kSwerveBRSpeed.id, Constants.CANDevices.kCANCoderBR.id,
      Constants.DriveConstants.backRightConstants,false,true);

  public static SwerveModule frontLeft = new SwerveModule(Constants.TalonID.kSwerveFLAngle.id,
      Constants.TalonID.kSwerveFLSpeed.id, Constants.CANDevices.kCANCoderFL.id,
      Constants.DriveConstants.frontLeftConstants,false,true);

  public static SwerveModule frontRight = new SwerveModule(Constants.TalonID.kSwerveFRAngle.id,
      Constants.TalonID.kSwerveFRSpeed.id, Constants.CANDevices.kCANCoderFR.id,
      Constants.DriveConstants.frontRightConstants,false,true);

  public static SwerveDrive swerveDrive = new SwerveDrive(backRight, backLeft, frontRight, frontLeft);

  public static Hopper hopper = new Hopper(Constants.TalonID.kLifterMotor.id, Constants.CANSparkMaxID.agitatorMotor.id);

  public static Hood hood = new Hood(Constants.TalonID.kHoodMotor.id);

  public static Intake intake = new Intake(Constants.CANSparkMaxID.intakeMotor.id);

  public static Shooter shooter = new Shooter(Constants.TalonID.kShooter1.id, Constants.TalonID.kShooter3.id,
      Constants.TalonID.kShooter2.id);

  public static DigitalInput beamBreakInp = new DigitalInput(0);

  public static BeamBreak beamBreak = new BeamBreak(beamBreakInp);

  public RobotContainer() {

    swerveDrive.setDefaultCommand(new Drive(swerveDrive));
    hopper.setDefaultCommand(new runHopper(hopper));
    beamBreak.setDefaultCommand(new beamIncrement(beamBreak));
    shooter.setDefaultCommand(new shoot(shooter));
    configureButtonBindings();

  }

  private void configureButtonBindings() {
   driver1A.toggleWhenPressed(new runIntake(intake));
   driver2RB.whileHeld(new forceShoot(hopper));
   
  }


  public static double getRightTriggerD2(){
    return driver2.getTriggerAxis(Hand.kRight);
  }

  public static double getXLeft() {
    double input = driver1.getX(Hand.kLeft);
    if (input < .125 && input > -.125) {
      return 0;
    } else {
      return input;
    }
  }

  public static double getYLeft() {
    double input = driver1.getY(Hand.kLeft);
    if(input < .075 && input > -.075){
      return 0;
    } else {
      return input;
    }
  }

  public static double getXRight() {

    double input = driver1.getX(Hand.kRight);
    if(input < .2 && input > -.2){
      return 0;
    } else {
      return input;
    }
  }
  /*
   * public Command getAutonomousCommand() {
   * 
   * return m_autoCommand; }
   */
}
