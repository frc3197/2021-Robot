package frc.robot.subsystems.Drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;

/** Represents a swerve drive style drivetrain. */
public class SwerveDrive {
    public static double maxSpeed = 0;
    public static double maxAngleSpeed = 0;

    // TODO: you must fix nums inside fool
    private final Translation2d m_frontLeftLocation = new Translation2d();
    private final Translation2d m_frontRightLocation = new Translation2d();
    private final Translation2d m_backLeftLocation = new Translation2d();
    private final Translation2d m_backRightLocation = new Translation2d();

    public final SwerveModule m_frontRight;
    public final SwerveModule m_frontLeft;
    public final SwerveModule m_backRight;
    public final SwerveModule m_backLeft;

    public static AHRS gyro = new AHRS(Port.kUSB);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation,
            m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, gyro.getRotation2d());

    public SwerveDrive(SwerveModule backRight,SwerveModule backLeft,SwerveModule frontRight,SwerveModule frontLeft) {
    m_frontRight = frontRight;
    m_frontLeft = frontLeft;
    m_backRight = backRight;
    m_backLeft = backLeft;


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