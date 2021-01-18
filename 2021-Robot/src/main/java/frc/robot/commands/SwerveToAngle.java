
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain.SwerveDrive;

public class SwerveToAngle extends CommandBase {
  SwerveDrive swerve;
  double angle;
  /**
   * Creates a new TurnToAngle.
   */
  public SwerveToAngle(SwerveDrive swerve, double angle) {
    this.swerve = swerve;
    this.angle = angle;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.turnAllToAngle(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean backLeft = angle - swerve.backLeft.getModuleAngle() == 0;
    boolean backRight = angle - swerve.backRight.getModuleAngle() == 0;
    boolean frontLeft = angle - swerve.frontLeft.getModuleAngle() == 0;
    boolean frontRight = angle - swerve.frontRight.getModuleAngle() == 0;
    if(backLeft && backRight && frontLeft && frontRight){
      return true;
    }
    else return false;
  }
}
