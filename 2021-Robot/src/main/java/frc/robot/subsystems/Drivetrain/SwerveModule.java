// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
// Measurments are all done in Meters.
  private static final double kWheelRadius = 0;
  private static final int kEncoderResolution = 0;

  private static final double kModuleMaxAngularVelocity = SwerveDrive.maxAngleSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final WPI_TalonFX speed_motor;
  private final WPI_TalonFX angle_motor;
  
  private final CANCoder encoder;
  private PIDController m_drivePIDController = new PIDController(1, 0, 0);

  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(1, 0, 0,
      new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  public SwerveModule(int angleMotor, int speedMotor, int encoderID) {
    angle_motor = new WPI_TalonFX(angleMotor);
    speed_motor = new WPI_TalonFX(speedMotor);
    encoder = new CANCoder(encoderID);
    m_drivePIDController = new PIDController(Constants.PIDContants.swerveModule.p, Constants.PIDContants.swerveModule.i,
        Constants.PIDContants.swerveModule.d);

    m_turningPIDController.setTolerance(5);
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getSpeedEncoderRate(){
    //Pulls the integrated sensor velocity
    double driveUnitsPer100ms = speed_motor.getSelectedSensorVelocity();
    // Converts the encoder rate to meters per second
    double encoderRate = driveUnitsPer100ms / Constants.talonEncoderResolution * 10 * Constants.swerveWheelDiam * Constants.swerveDriveMotorGR;
    return encoderRate;
  }

  public double getAngle(){
    return encoder.getAbsolutePosition();
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getSpeedEncoderRate(),new Rotation2d(getAngle()));
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
        optimize(desiredState, new Rotation2d(getAngle()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(getSpeedEncoderRate(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(getAngle(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    speed_motor.setVoltage(driveOutput + driveFeedforward);
    angle_motor.setVoltage(turnOutput + turnFeedforward);



  }


}
