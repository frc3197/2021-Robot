// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Vision.calibrateHood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.SwerveDrive;

public class Robot extends TimedRobot {
  public static double autoStartingGyro = 0;
  private Command resetHood = new calibrateHood(RobotContainer.hood);
  private RobotContainer m_robotContainer;

  private Command m_autonomousCommand;
  @Override
  public void robotInit() {

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    
    m_robotContainer = new RobotContainer();
    System.out.print(m_robotContainer.hood.getLimitSwitchState());
    
   //m_robotContainer.hood.calibrateEncoderPosition();
    //SwerveDrive.gyro.calibrate();
    //SwerveDrive.gyro.reset();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    autoStartingGyro = SwerveDrive.gyro.getAngle();
    m_autonomousCommand = m_robotContainer.getSwerveControllerPath();


    /*
    GALACTIC SEARCH CHOOSER
    double gyroAng = RobotContainer.swerveDrive.gyro.getAngle();
    double ballOffset = RobotContainer.intake.getYaw();
    if(gyroAng == 0){

      if(ballOffset == 0){
        RUN PATH: RED A
      }
      else if(ballOffset < -10{
        RUN PATH: RED B
      }

    if(gyroAng < 100 && gyroAng > 80){
      RUN FORWARD TO D5
      THEN CHECK

      if(ballOffset == 0){
        RUN PATH: BLUE B
      }
      else if (ballOffset > 10){
        RUN PATH: BLUE B
      }
    }


    }
    */
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {

    
    RobotContainer.intake.getCam().setDriverMode(true);
    RobotContainer.swerveDrive.ResetOdometry(new Pose2d(0,0,new Rotation2d(Units.degreesToRadians(180))));
    RobotContainer.swerveDrive.resetEncoders();
    RobotContainer.swerveDrive.resetGyro();
    RobotContainer.hood.resetEncoder();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(Shooter.getShooterPIDController().atSetpoint()){
      m_robotContainer.getDriver2().setRumble(RumbleType.kLeftRumble, 1);
      m_robotContainer.getDriver2().setRumble(RumbleType.kRightRumble, 1); 
    }
    else{
      
      m_robotContainer.getDriver2().setRumble(RumbleType.kLeftRumble, 0);
      m_robotContainer.getDriver2().setRumble(RumbleType.kRightRumble, 0); 
    }
    
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
