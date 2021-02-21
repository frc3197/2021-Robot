// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Mechs.Continuous;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.SwerveDrive;

public class Drive extends CommandBase {
  
  private final SwerveDrive m_swerve;

  private boolean fieldRelative = true;

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(4);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(4);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(20);


  double x1,x2,y1;

  /**
   * Creates a new Drive.
   */

   public Drive(SwerveDrive m_swerve){
     this.m_swerve = m_swerve;
     addRequirements(m_swerve);
    
   }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x1 = -RobotContainer.getXLeftController();
    x2 = -RobotContainer.getXRightController();
    y1 = RobotContainer.getYLeftController();
  
      // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
     var xSpeed =
        m_xspeedLimiter.calculate(y1)
            * SwerveDrive.maxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
     var ySpeed =
        -m_yspeedLimiter.calculate(x1)
            * SwerveDrive.maxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
     var rot =
        -m_rotLimiter.calculate(x2)
            * SwerveDrive.maxAngleSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.drive(0, 0, 0, fieldRelative);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
