// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.TrajectoryLookupTable;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class bluePathLogic extends InstantCommand {
  TrajectoryLookupTable trajectoryLookupTable;
  double yawOffset;
  public bluePathLogic(TrajectoryLookupTable trajectoryLookupTable) {
    this.trajectoryLookupTable = trajectoryLookupTable;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(yawOffset > -55 || yawOffset < -35){
      trajectoryLookupTable.getBLUE_A().schedule();
    }
    else if(yawOffset < 10 || yawOffset > -10){
    trajectoryLookupTable.getBLUE_B().schedule();
    }

  }
}
