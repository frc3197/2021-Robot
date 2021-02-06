// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
// Measurments are all done in Meters.


  private static final double kModuleMaxAngularVelocity = SwerveDrive.maxAngleSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final WPI_TalonFX speed_motor;
  private final WPI_TalonFX angle_motor;
  
  private final CANCoder encoder;
//TODO: Set Proper Constant Values: PID Drive Controller
  private PIDController m_drivePIDController = new PIDController(0, 0, 0);

  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(Constants.PIDContants.swerveAnge.p, Constants.PIDContants.swerveAnge.i, Constants.PIDContants.swerveAnge.d,
      new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

//TODO: Set Proper Constant Values: FeedForward Constants  
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(.8,Constants.angleFeedForwardkV);

  public SwerveModule(int angleMotor, int speedMotor, int encoderID) {
    angle_motor = new WPI_TalonFX(angleMotor);
    speed_motor = new WPI_TalonFX(speedMotor);

    encoder = new CANCoder(encoderID);
    m_drivePIDController = new PIDController(Constants.PIDContants.swerveSpeed.p, Constants.PIDContants.swerveSpeed.i,
        Constants.PIDContants.swerveSpeed.d);
    angle_motor.configOpenloopRamp(0);
    speed_motor.configOpenloopRamp(.2);
    m_turningPIDController.setTolerance(2);
    m_drivePIDController.setTolerance(5);
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getRPM(){
    return angle_motor.getSelectedSensorVelocity() / Constants.talonEncoderResolution * 10 * 60;
  }

  public double getSpeedEncoderRate(){
    //Pulls the integrated sensor velocity
    double driveUnitsPer100ms = speed_motor.getSelectedSensorVelocity();
    // Converts the encoder rate to meters per second
    double encoderRate = driveUnitsPer100ms / Constants.talonEncoderResolution * 10 * Constants.swerveWheelDiam * Constants.swerveDriveMotorGR;
    return encoderRate;
  }

  public double getAngleRadians(){
    return Units.degreesToRadians(encoder.getAbsolutePosition());
  }

  public void resetDriveEncoder(){
    speed_motor.setSelectedSensorPosition(0);
  }

  public void setVoltageAngle(double voltage){
    angle_motor.setVoltage(voltage);
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

    speed_motor.setVoltage(driveOutput + driveFeedforward);
    angle_motor.setVoltage(turnOutput + turnFeedforward);



  }


}
