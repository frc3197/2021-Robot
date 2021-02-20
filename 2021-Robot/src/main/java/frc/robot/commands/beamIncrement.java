// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.Hopper;

public class beamIncrement extends CommandBase {
  /** Creates a new beamBreak. */
  static boolean beamNotBroken;
  boolean alreadyAdded = false;
  BeamBreak beamBreak;
  boolean temp;
  boolean change = true; 

  public beamIncrement(BeamBreak beamBreak) {
  
  this.beamBreak = beamBreak;
  addRequirements(beamBreak);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    temp = beamNotBroken;
    beamNotBroken = beamBreak.getBeamBreakState();
    SmartDashboard.putBoolean("BeamBreak", beamNotBroken);
    
    if(beamNotBroken != temp){
      change = true;
    }
    else{
      change = false;
    }


  if(change == true){

    if (beamNotBroken == false) {
      Hopper.count += 1;
      alreadyAdded = true;
    
    } else if (alreadyAdded && beamNotBroken ) {
      Hopper.count -= 1;
    } else {}
  
  }}

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
