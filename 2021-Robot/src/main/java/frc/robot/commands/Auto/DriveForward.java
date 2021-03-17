// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain.SwerveDrive;

public class DriveForward extends CommandBase {
  SwerveDrive swerveDrive;
  double distance;
  double initialDistance;
  double currentDistance;
  double output;
  PIDController pidController;

  /** Creates a new DriveForward. */
  public DriveForward(double distance, SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
    this.distance = distance;
    pidController = new PIDController(0, 0, 0);
    addRequirements(swerveDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialDistance = swerveDrive.returnOdometry().getPoseMeters().getX();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentDistance = swerveDrive.returnOdometry().getPoseMeters().getX();
    output = pidController.calculate(currentDistance - initialDistance, distance);
    swerveDrive.drive(output, 0, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
