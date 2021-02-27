// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Mechs.Button.Agitate;
import frc.robot.commands.Mechs.Button.forceShoot;
import frc.robot.commands.Mechs.Button.moveHood;
import frc.robot.commands.Mechs.Button.runIntakeHopper;
import frc.robot.commands.Mechs.Continuous.Drive;
import frc.robot.commands.Mechs.Continuous.shoot;
import frc.robot.commands.Sequential.IntakeAlignDriveSequence;
import frc.robot.commands.Vision.DriverModeToggle;
import frc.robot.commands.Vision.intakeAlign;
import frc.robot.commands.Vision.intakeAlign2;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.SwerveDrive;
import frc.robot.subsystems.Drivetrain.SwerveModule;

public class RobotContainer {
  

  private static Joystick thrustmaster = new Joystick(2);
  private static XboxController driver1 = new XboxController(0);
  private static XboxController driver2 = new XboxController(1);

  private JoystickButton driver1A = new JoystickButton(driver1, 1);
  private JoystickButton driver1X = new JoystickButton(driver1, 3);
  private JoystickButton driver1Y = new JoystickButton(driver1, 4);
  private JoystickButton driver1B = new JoystickButton(driver1, 2);
  private JoystickButton driver1Start = new JoystickButton(driver1, 8);

  
  private JoystickButton driver2RB = new JoystickButton(driver2, 6);
  private JoystickButton driver2X = new JoystickButton(driver2, 3);
  private JoystickButton driver2Y = new JoystickButton(driver2, 2);
  private JoystickButton driver2A = new JoystickButton(driver2, 1);

  private JoystickButton driver1JFwdInt = new JoystickButton(thrustmaster, 1);
  private JoystickButton driver1JRevInt = new JoystickButton(thrustmaster,5);

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
  public static Agitator agitator = new Agitator(Constants.CANSparkMaxID.agitatorMotor.id);

  public static DigitalInput beamBreakInp = new DigitalInput(0);

  public static BeamBreak beamBreak = new BeamBreak(beamBreakInp);

  public RobotContainer() {
    
    swerveDrive.setDefaultCommand(new Drive(swerveDrive));
    shooter.setDefaultCommand(new shoot(shooter));
    configureButtonBindings();

  }

  private void configureButtonBindings() {
   driver1A.toggleWhenPressed(new runIntakeHopper(intake,hopper,1));
   driver1Start.whileHeld(new runIntakeHopper(intake, hopper, -1));
   driver1X.whenPressed(new IntakeAlignDriveSequence(intake, swerveDrive));
   driver1X.whenReleased(new DriverModeToggle(false));

   driver1Y.whenPressed(new DriverModeToggle(false));
   driver1Y.whileHeld(new intakeAlign2(swerveDrive,intake));
   driver1Y.whenReleased(new DriverModeToggle(true));

   driver2RB.whileHeld(new forceShoot(hopper));
   driver2X.whileHeld(new moveHood(hood, -1 * Constants.MotorOutputMultiplier.hood.multiplier));
   driver2Y.whileHeld(new moveHood(hood,  1 * Constants.MotorOutputMultiplier.hood.multiplier));
   driver2A.toggleWhenPressed(new Agitate(agitator));
  }


  public static double getRightTriggerD2(){
    return driver2.getTriggerAxis(Hand.kRight);
  }

  public static double getXLeftController() {
    double input = driver1.getX(Hand.kLeft);
    if (input < .125 && input > -.125) {
      return 0;
    } else {
      return input;
    }
  }

  public static double getYLeftController() {
    double input = driver1.getY(Hand.kLeft);
    if(input < .075 && input > -.075){
      return 0;
    } else {
      return input;
    }
  }

  public static double getXRightController() {

    double input = driver1.getX(Hand.kRight);
    if(input < .1 && input > -.1){
      return 0;
    } else {
      return input;
    }
  }
  
  public static double getXLeftJoystick() {
    double input = thrustmaster.getX();
    if (input < .125 && input > -.125) {
      return 0;
    } else {
      return input;
    }
  }

  public static double getYLeftJoystick() {
    double input = thrustmaster.getY();
    if(input < .075 && input > -.075){
      return 0;
    } else {
      return input;
    }
  }

  public static double getXRightJoystick() {

    double input = thrustmaster.getZ();
    if(input < .2 && input > -.2){
      return 0;
    } else {
      return input;
    }
  }
  public static double getDistanceFromTarget() {
    double ty = NetworkTableInstance.getDefault().getTable("limelight-killroy").getEntry("ty").getDouble(0);
    SmartDashboard.putNumber("ty", ty);
    double offset = Units.degreesToRadians(Constants.limelightOffsetDegrees);
    ty = Units.degreesToRadians(ty);
    double limeDistance = Math.abs(Constants.heightOfPP / (Math.tan(ty + offset)));
    SmartDashboard.putNumber("Distance from Target", limeDistance);
    if(limeDistance < 270 && limeDistance > 30){
      SmartDashboard.putBoolean("Inside Shooting Range", true);
    }else{
      SmartDashboard.putBoolean("Inside Shooting Range", false);
    }
    return limeDistance;
    // Will have to integrate a variable a1 value once set up for limelight angle.
  }
  public static boolean isBetween(int x, int value) {
    return (value - 5) <= x && x <= (value + 5);
  }
  /*
   * public Command getAutonomousCommand() {
   * 
   * return m_autoCommand; }
   */
}
