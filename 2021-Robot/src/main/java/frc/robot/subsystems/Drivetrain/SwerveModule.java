// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule extends SubsystemBase {
// Measurments are all done in Meters.


  private static final double kModuleMaxAngularVelocity = SwerveDrive.maxAngleSpeed;
  private static final double kModuleMaxAngularAcceleration = 22.5 * Math.PI; // radians per second squared


  private final WPI_TalonFX speed_motor;
  private final WPI_TalonFX angle_motor;
  
  private SwerveModuleConstants swerveModuleConstants;
  static int numModule = 0;
  
  final int moduleNumber;
  private final CANCoder encoder;
  private PIDController m_drivePIDController;

  private final ProfiledPIDController m_turningPIDController;

  private final SimpleMotorFeedforward m_driveFeedforward;
  private final SimpleMotorFeedforward m_turnFeedforward;

  public SwerveModule(int angleMotor, int speedMotor, int encoderID, SwerveModuleConstants swerveModuleConstants , boolean angleInverted, boolean speedInverted) {
    moduleNumber = numModule;
    numModule++;
    angle_motor = new WPI_TalonFX(angleMotor);
    speed_motor = new WPI_TalonFX(speedMotor);
    this.swerveModuleConstants = swerveModuleConstants;

    speed_motor.configFactoryDefault();
    angle_motor.configFactoryDefault();

    m_turningPIDController = new ProfiledPIDController(swerveModuleConstants.P_angle,0, swerveModuleConstants.D_angle,
      new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    m_drivePIDController = new PIDController(swerveModuleConstants.P_drive, 0, swerveModuleConstants.D_drive);

    m_driveFeedforward = new SimpleMotorFeedforward(swerveModuleConstants.kS_drive,swerveModuleConstants.kV_drive);
    m_turnFeedforward = new SimpleMotorFeedforward(swerveModuleConstants.kS_angle,swerveModuleConstants.kV_angle);
    encoder = new CANCoder(encoderID);
  
    angle_motor.configOpenloopRamp(0);
    speed_motor.configOpenloopRamp(.035);
    angle_motor.setNeutralMode(NeutralMode.Brake);
    speed_motor.setNeutralMode(NeutralMode.Brake);

    angle_motor.setInverted(angleInverted);
    speed_motor.setInverted(speedInverted);
    m_turningPIDController.setTolerance(0);
    m_drivePIDController.setTolerance(3);
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }

  public double getRPM(){
    return speed_motor.getSelectedSensorVelocity() / Constants.talonEncoderResolution * 10 * 60;
  }

  public double getSpeedEncoderRate(){
    //Pulls the integrated sensor velocity
    double driveUnitsPer100ms = speed_motor.getSelectedSensorVelocity();
    // Converts the encoder rate to meters per second
    double encoderRate = driveUnitsPer100ms / Constants.talonEncoderResolution * 10 * Constants.swerveWheelCircumMeters * Constants.swerveDriveMotorGR;
    SmartDashboard.putNumber("SpeedEncoderRate", encoderRate);
    return encoderRate;
  }

  public double getAngleRadians(){
    return Units.degreesToRadians(encoder.getAbsolutePosition());
  }

  public void resetDriveEncoder(){
    speed_motor.setSelectedSensorPosition(0);
  }

  public void setVoltageSpeed(double voltage){
    speed_motor.setVoltage(voltage);
  }

  public SwerveModuleConstants getConstants(){
    return swerveModuleConstants;
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getSpeedEncoderRate(),new Rotation2d(getAngleRadians()));
  }

  public SwerveModuleState optimize(
    SwerveModuleState desiredState, Rotation2d currentAngle) {
  var delta = desiredState.angle.minus(currentAngle);
  if (Math.abs(delta.getDegrees()) > 90.0) {
    return new SwerveModuleState(
        -desiredState.speedMetersPerSecond,
        desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
  } else {
    return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
  }
}





  public void setDesiredState(SwerveModuleState desiredState){

    SwerveModuleState state =
        optimize(desiredState, new Rotation2d(getAngleRadians()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(getSpeedEncoderRate(), state.speedMetersPerSecond);




    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);



    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(getAngleRadians(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    speed_motor.setVoltage((driveOutput + driveFeedforward) * RobotController.getBatteryVoltage());

    //angle_motor.setVoltage(0);
    angle_motor.setVoltage((turnOutput + turnFeedforward) * RobotController.getBatteryVoltage());
  }


}
