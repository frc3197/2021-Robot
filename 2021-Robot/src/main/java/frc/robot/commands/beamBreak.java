// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;

public class beamBreak extends CommandBase {
  /** Creates a new beamBreak. */
  static boolean beamBroken;
  boolean alreadyAdded = false;

  DigitalInput input;
  

  public beamBreak(DigitalInput input){
  
  this.input = input;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    beamBroken = input.get();

    if (beamBroken == true) {
      Hopper.count += 1;
      alreadyAdded = true;
    } else if (alreadyAdded && !beamBroken) {
      Hopper.count -= 1;
    } else {
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
