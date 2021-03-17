package frc.robot;

import java.io.IOException;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain.SwerveDrive;

public class TrajectoryLookupTable {
private SwerveDrive swerveDrive;
    
private Trajectory galacticSearch_Red_A1;
private Trajectory galacticSearch_Red_A2;
private Trajectory galacticSearch_Red_A3;
private Trajectory galacticSearch_Red_A4;

private Trajectory galacticSearch_Red_B1;
private Trajectory galacticSearch_Red_B2;
private Trajectory galacticSearch_Red_B3;
private Trajectory galacticSearch_Red_B4;

private Trajectory galacticSearch_Blue_A1;
private Trajectory galacticSearch_Blue_A2;
private Trajectory galacticSearch_Blue_A3;
private Trajectory galacticSearch_Blue_A4;

private Trajectory galacticSearch_Blue_B1;
private Trajectory galacticSearch_Blue_B2;
private Trajectory galacticSearch_Blue_B3;
private Trajectory galacticSearch_Blue_B4;

private String galacticSearch_Red_A_JSON1 = "";
private String galacticSearch_Red_A_JSON2 = "";
private String galacticSearch_Red_A_JSON3 = "";
private String galacticSearch_Red_A_JSON4 = "";

private String galacticSearch_Red_B_JSON1 = "";
private String galacticSearch_Red_B_JSON2 = "";
private String galacticSearch_Red_B_JSON3 = "";
private String galacticSearch_Red_B_JSON4 = "";

private String galacticSearch_Blue_A_JSON1 = "";
private String galacticSearch_Blue_A_JSON2 = "";
private String galacticSearch_Blue_A_JSON3 = "";
private String galacticSearch_Blue_A_JSON4 = "";

private String galacticSearch_Blue_B_JSON1 = "";
private String galacticSearch_Blue_B_JSON2 = "";
private String galacticSearch_Blue_B_JSON3 = "";
private String galacticSearch_Blue_B_JSON4 = "";

private ProfiledPIDController profliedPID;

public TrajectoryLookupTable(SwerveDrive swerveDrive){
this.swerveDrive = swerveDrive;

 profliedPID = new ProfiledPIDController(AutoConstants.thetaPIDConstants.kP.constant, AutoConstants.thetaPIDConstants.kI.constant, AutoConstants.thetaPIDConstants.kD.constant,
new TrapezoidProfile.Constraints(AutoConstants.maxAngleSpeed, AutoConstants.maxAngleAcceleration));
profliedPID.setTolerance(Units.degreesToRadians(3));
profliedPID.enableContinuousInput(-Math.PI, Math.PI);

{
try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(galacticSearch_Red_A_JSON1);
    galacticSearch_Red_A1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
} catch (IOException ex) {
    DriverStation.reportError("Unable to open trajectory: " + galacticSearch_Red_A_JSON1, ex.getStackTrace());
}




try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(galacticSearch_Red_A_JSON2);
    galacticSearch_Red_A2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
} catch (IOException ex1) {
    DriverStation.reportError("Unable to open trajectory: " + galacticSearch_Red_A_JSON2, ex1.getStackTrace());
}




try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(galacticSearch_Red_A_JSON3);
    galacticSearch_Red_A3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
} catch (IOException ex2) {
    DriverStation.reportError("Unable to open trajectory: " + galacticSearch_Red_A_JSON3, ex2.getStackTrace());
}




try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(galacticSearch_Red_A_JSON4);
    galacticSearch_Red_A4 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
} catch (IOException ex3) {
    DriverStation.reportError("Unable to open trajectory: " + galacticSearch_Red_A_JSON4, ex3.getStackTrace());
}




try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(galacticSearch_Red_B_JSON1);
    galacticSearch_Red_B1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
} catch (IOException ex4) {
    DriverStation.reportError("Unable to open trajectory: " + galacticSearch_Red_B_JSON1, ex4.getStackTrace());
}


try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(galacticSearch_Red_B_JSON2);
    galacticSearch_Red_B2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
} catch (IOException ex5) {
    DriverStation.reportError("Unable to open trajectory: " + galacticSearch_Red_B_JSON2, ex5.getStackTrace());
}


try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(galacticSearch_Red_B_JSON3);
    galacticSearch_Red_B3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
} catch (IOException ex6) {
    DriverStation.reportError("Unable to open trajectory: " + galacticSearch_Red_B_JSON3, ex6.getStackTrace());
}


try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(galacticSearch_Red_B_JSON4);
    galacticSearch_Red_B4 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
} catch (IOException ex7) {
    DriverStation.reportError("Unable to open trajectory: " + galacticSearch_Red_B_JSON4, ex7.getStackTrace());
}


try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(galacticSearch_Blue_A_JSON1);
    galacticSearch_Blue_A1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
} catch (IOException ex8) {
    DriverStation.reportError("Unable to open trajectory: " + galacticSearch_Blue_A_JSON1, ex8.getStackTrace());
}




try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(galacticSearch_Blue_A_JSON2);
    galacticSearch_Blue_A2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
} catch (IOException ex9) {
    DriverStation.reportError("Unable to open trajectory: " + galacticSearch_Blue_A_JSON2, ex9.getStackTrace());
}




try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(galacticSearch_Blue_A_JSON3);
    galacticSearch_Blue_A3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
} catch (IOException ex10) {
    DriverStation.reportError("Unable to open trajectory: " + galacticSearch_Blue_A_JSON3, ex10.getStackTrace());
}




try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(galacticSearch_Blue_A_JSON4);
    galacticSearch_Blue_A4 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
} catch (IOException ex11) {
    DriverStation.reportError("Unable to open trajectory: " + galacticSearch_Blue_A_JSON4, ex11.getStackTrace());
}




try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(galacticSearch_Blue_B_JSON1);
    galacticSearch_Blue_B1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
} catch (IOException ex12) {
    DriverStation.reportError("Unable to open trajectory: " + galacticSearch_Blue_B_JSON1, ex12.getStackTrace());
}


try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(galacticSearch_Blue_B_JSON2);
    galacticSearch_Blue_B2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
} catch (IOException ex13) {
    DriverStation.reportError("Unable to open trajectory: " + galacticSearch_Blue_B_JSON2, ex13.getStackTrace());
}


try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(galacticSearch_Blue_B_JSON3);
    galacticSearch_Blue_B3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
} catch (IOException ex14) {
    DriverStation.reportError("Unable to open trajectory: " + galacticSearch_Blue_B_JSON3, ex14.getStackTrace());
}


try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(galacticSearch_Blue_B_JSON4);
    galacticSearch_Blue_B4 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
} catch (IOException ex15) {
    DriverStation.reportError("Unable to open trajectory: " + galacticSearch_Blue_B_JSON4, ex15.getStackTrace());
}

}}


public SwerveControllerCommand createCommand(Trajectory trajectory) {
    
SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    trajectory , // Trajectory being used 
    swerveDrive::getPose, // Supplier for position on the field
    swerveDrive.getKinematics(), // SwerveKinematics Object
    new PIDController(AutoConstants.xPIDConstants.kP.constant, AutoConstants.xPIDConstants.kI.constant, AutoConstants.xPIDConstants.kD.constant),  // X Controller
    new PIDController(AutoConstants.yPIDConstants.kP.constant, AutoConstants.yPIDConstants.kI.constant, AutoConstants.yPIDConstants.kD.constant), // Y Controller
    profliedPID, // Theta Controller
    swerveDrive::setModuleStates, // Consumer of swerveModuleStates
    swerveDrive); // Subsystem Requirements
   
   // Reset odometry to the starting pose of the trajectory.
   swerveDrive.ResetOdometry(trajectory.getInitialPose());

   return swerveControllerCommand;
   }
    





public SequentialCommandGroup getRED_A(){
SwerveControllerCommand redA1Command = createCommand(galacticSearch_Red_A1);
SwerveControllerCommand redA2Command = createCommand(galacticSearch_Red_A2);
SwerveControllerCommand redA3Command = createCommand(galacticSearch_Red_A3);
SwerveControllerCommand redA4Command = createCommand(galacticSearch_Red_A4);

    return new SequentialCommandGroup(redA1Command,redA2Command,redA3Command,redA4Command);
}
public SequentialCommandGroup getRED_B(){
    SwerveControllerCommand redB1Command = createCommand(galacticSearch_Red_B1);
    SwerveControllerCommand redB2Command = createCommand(galacticSearch_Red_B2);
    SwerveControllerCommand redB3Command = createCommand(galacticSearch_Red_B3);
    SwerveControllerCommand redB4Command = createCommand(galacticSearch_Red_B4);
    
        return new SequentialCommandGroup(redB1Command,redB2Command,redB3Command,redB4Command);
}
public SequentialCommandGroup getBLUE_A(){
    SwerveControllerCommand blueA1Command = createCommand(galacticSearch_Blue_A1);
    SwerveControllerCommand blueA2Command = createCommand(galacticSearch_Blue_A2);
    SwerveControllerCommand blueA3Command = createCommand(galacticSearch_Blue_A3);
    SwerveControllerCommand blueA4Command = createCommand(galacticSearch_Blue_A4);
    
        return new SequentialCommandGroup(blueA1Command,blueA2Command,blueA3Command,blueA4Command);
}
public SequentialCommandGroup getBLUE_B(){
    SwerveControllerCommand blueB1Command = createCommand(galacticSearch_Blue_B1);
    SwerveControllerCommand blueB2Command = createCommand(galacticSearch_Blue_B2);
    SwerveControllerCommand blueB3Command = createCommand(galacticSearch_Blue_B3);
    SwerveControllerCommand blueB4Command = createCommand(galacticSearch_Blue_B4);
    
        return new SequentialCommandGroup(blueB1Command,blueB2Command,blueB3Command,blueB4Command);
}



}