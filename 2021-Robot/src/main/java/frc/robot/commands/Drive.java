// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.SwerveDrive;

public class Drive extends CommandBase {
private SwerveDrive swerve;
double x1;
double y1;
double x2;
  /**
   * Creates a new Drive.
   */
  public Drive(SwerveDrive swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x1 = RobotContainer.getXLeft();
    y1 = RobotContainer.getYLeft();
    x2 = RobotContainer.getXRight();
    SmartDashboard.putNumber("X left", x1);
    SmartDashboard.putNumber("Y Left", y1);
    SmartDashboard.putNumber("X Right", x2);
    swerve.driveRoboCentric(x1, y1, x2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.driveFieldCentric(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
