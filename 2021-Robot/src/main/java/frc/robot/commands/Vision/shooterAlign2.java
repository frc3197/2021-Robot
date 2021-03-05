// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.SwerveDrive;

public class shooterAlign2 extends CommandBase {
  PIDController pidController;
  Shooter shooter;
  SwerveDrive swerveDrive;
  double measurement;
  double output;
  /** Creates a new shooterAlign2. */
  public shooterAlign2(Shooter shooter, SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
    this.shooter = shooter;
    addRequirements(shooter,swerveDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pidController = shooter.getAlignPIDController();
    measurement = shooter.getXOffset();
    SmartDashboard.putNumber("x offset ll", measurement);
    output = pidController.calculate(measurement, 0);
    swerveDrive.drive(0, 0, output * -.1, true);
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {swerveDrive.drive(0, 0, 0, true);}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
