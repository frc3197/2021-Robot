// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Debug;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain.SwerveDrive;
import frc.robot.subsystems.Drivetrain.SwerveModule;

public class SetVoltage extends CommandBase {
  SwerveDrive swerve;
  double voltage;
  /** Creates a new SetVoltage. */
  public SetVoltage(SwerveDrive swerve, double voltage) {
    this.swerve = swerve;
    this.voltage = voltage;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.setVoltageAllMotors(voltage);
    SmartDashboard.putNumber("SetVoltage BL", swerve.m_backLeft.getRPM());
    SmartDashboard.putNumber("SetVoltage BR", swerve.m_backRight.getRPM());
    SmartDashboard.putNumber("SetVoltage FL", swerve.m_frontLeft.getRPM());
    SmartDashboard.putNumber("SetVoltage FR", swerve.m_frontRight.getRPM());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setVoltageAllMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
