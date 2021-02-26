// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drivetrain.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class intakeAlign extends PIDCommand {
  Intake intake;
  SwerveDrive swerve;
  /** Creates a new intakeAlign. */


  public intakeAlign(Intake intake, SwerveDrive swerve) {
    super(
        // The controller that the command will use
        // TODO: Set Proper Constant Values: Intake PID Align
        new PIDController(.2, 0, 0),
        // This should return the measurement
        intake::getYaw,
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        output -> {
          swerve.drive(0, 0, output * -.3, true);
          // Use the output here
        //swerve.driveRoboCentric(0, 0, output * .4);



        });
      getController().setTolerance(360,360);
      addRequirements(intake, swerve);
      this.intake = intake;
      this.swerve = swerve;

    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
