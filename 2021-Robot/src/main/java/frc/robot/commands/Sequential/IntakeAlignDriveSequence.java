// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequential;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Vision.DriverModeToggle;
import frc.robot.commands.Vision.intakeAlign;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drivetrain.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAlignDriveSequence extends SequentialCommandGroup {
  SwerveDrive swerve;
  Intake intake;
  /** Creates a new IntakeAlignDriveSequence. */
  public IntakeAlignDriveSequence(Intake intake, SwerveDrive swerve) {
    this.intake = intake;
    this.swerve = swerve;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriverModeToggle(false),new intakeAlign(intake, swerve));
  }
}
