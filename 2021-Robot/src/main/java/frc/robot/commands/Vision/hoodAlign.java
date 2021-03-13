// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.ShooterLookupTable;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class hoodAlign extends CommandBase {
  Hood hood;
  double output;
  double targetEncoderValue;
  double distanceFromTarget;
  double currentEncoderValue;
  /** Creates a new hoodAlign. */
  public hoodAlign(Hood hood) {
    this.hood = hood;
    addRequirements(hood);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentEncoderValue = hood.getEncoderPosition();
    distanceFromTarget = RobotContainer.getDistanceFromTarget();
    targetEncoderValue = ShooterLookupTable.lookupEncoderTarget((int)distanceFromTarget);
    output = hood.getPIDController().calculate((targetEncoderValue - currentEncoderValue) * .004,0);
    hood.setHood(-output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.setHood(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (currentEncoderValue == targetEncoderValue);
  }
}
