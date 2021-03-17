// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain.SwerveDrive;

public class TurnAngle extends CommandBase {
  double angle;
  double initialAngle;
  double targetAngle;
  double currentAngle;
  double output;

  SwerveDrive swerveDrive;

  PIDController pidController;
  /** Creates a new TurnAngle. */
  public TurnAngle(double angle, SwerveDrive swerveDrive) {
    this.angle = angle;
    this.swerveDrive = swerveDrive;
  
    pidController = new PIDController(0, 0, 0);
    pidController.setTolerance(3);

    addRequirements(swerveDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = SwerveDrive.gyro.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = SwerveDrive.gyro.getAngle();
    output = pidController.calculate(currentAngle-initialAngle, targetAngle);
    swerveDrive.drive(0, 0, output, true);
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
