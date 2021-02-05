package frc.robot.subsystems.Drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Represents a swerve drive style drivetrain. */
public class SwerveDrive {
    public static double maxSpeed = 0;
    public static double maxAngleSpeed = 0;

    // TODO: you must fix nums inside fool
    private final Translation2d m_frontLeftLocation = new Translation2d();
    private final Translation2d m_frontRightLocation = new Translation2d();
    private final Translation2d m_backLeftLocation = new Translation2d();
    private final Translation2d m_backRightLocation = new Translation2d();

    public final SwerveModule m_frontRight = new SwerveModule();
    public final SwerveModule m_frontLeft = new SwerveModule();
    public final SwerveModule m_backRight = new SwerveModule();
    public final SwerveModule m_backLeft = new SwerveModule();

    public static AHRS gyro = new AHRS(Port.kUSB);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation,
            m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, gyro.getRotation2d());

    public SwerveDrive() {
    gyro.reset();
}
    //drive command
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates =
            m_kinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, maxSpeed);
        //TODO: 
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[0]);
        m_backLeft.setDesiredState(swerveModuleStates[0]);
        m_backRight.setDesiredState(swerveModuleStates[0]);
      }
//do things again 
    public void updateOdometry() {
        m_odometry.update(
            gyro.getRotation2d(),
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState());
      }
}