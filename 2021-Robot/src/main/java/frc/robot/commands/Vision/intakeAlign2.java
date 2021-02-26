// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drivetrain.SwerveDrive;

public class intakeAlign2 extends CommandBase {
  SwerveDrive swerve;
  Intake intake;
  double measurement;
  double output;
  PIDController pidController;
  /** Creates a new intakeAlign2. */
  public intakeAlign2(SwerveDrive swerve, Intake intake) {
    this.intake = intake;
    this.swerve = swerve;
    addRequirements(swerve,intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pidController = intake.getPIDController();
    boolean targetExist = Intake.getCam().getLatestResult().hasTargets();
    measurement = intake.getYaw();
    output = pidController.calculate(measurement, 0);

    if(targetExist){
    swerve.drive(0, 0, output * -1, true);
    }
    else{
      swerve.drive(0, 0, -1, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
