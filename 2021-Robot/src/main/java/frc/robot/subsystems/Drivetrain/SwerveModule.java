// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

  private WPI_TalonFX angle_motor;
  private WPI_TalonFX speed_motor;

  // private double mLastTargetAngle;
  private double mZeroOffset;

	
  // Voltage cap
  private double MAX_VOLTS;

  // Creates a PIDController for use in turning to an angle
  private PIDController pidController;
  // Creates an encoder for judging angle
  private CANCoder encoder;
  /**
   * Creates a new WheelDrive.
   */
  public SwerveModule(int angleMotor, int speedMotor, int encoderID, int maxVoltage) {
    angle_motor = new WPI_TalonFX(angleMotor);
    speed_motor = new WPI_TalonFX(speedMotor);
    encoder = new CANCoder(encoderID);

    pidController = new PIDController(0, 0, 0);
    pidController = new PIDController(Constants.PIDContants.swerveModule.p, Constants.PIDContants.swerveModule.i, Constants.PIDContants.swerveModule.d);
    MAX_VOLTS = maxVoltage;
    

    pidController.setTolerance(5);
    angle_motor.setNeutralMode(NeutralMode.Brake);
    speed_motor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
	public double getModuleAngleAbsolute(){
  double currentAngle = encoder.getAbsolutePosition();
  return currentAngle;
  }


  // Absolute useless garbage

	public double getModuleAngle(){
    double currentAngle = encoder.getPosition() % 360;
    return currentAngle;
    }


	
  public void turnToAngle(double targetAngle){
    double currentAngle = encoder.getAbsolutePosition();
 		double delta = currentAngle - targetAngle;
 		if (delta > 90 || delta < -90) {
 			if (delta > 90)
 				targetAngle += 180;
 			else if (delta < -90){
 				targetAngle -= 180;
 			speed_motor.setInverted(false);
 		} else {
 			speed_motor.setInverted(true);
 		}}
	  
		 // targetAngle += currentAngle - currentAngleMod;
    // ALL OF THIS STUFF IS FOR DEALING WITH STALLING, SHOULD DISECT LATER 
		// double currentError = angleMotor.getError();
		// if (Math.abs(currentError - mLastError) < 7.5 &&
		// 		Math.abs(currentAngle - targetAngle) > 5) {
		// 	if (mStallTimeBegin == Long.MAX_VALUE) mStallTimeBegin = System.currentTimeMillis();
		// 	if (System.currentTimeMillis() - mStallTimeBegin > STALL_TIMEOUT) {
		// 		throw new MotorStallException(String.format("Angle motor on swerve module '%d' has stalled.",
		// 				mModuleNumber));
		// 	}
		// } else {
		// 	mStallTimeBegin = Long.MAX_VALUE;
		// }
		// mLastError = currentError;


		pidController.setSetpoint(targetAngle);
	}
  
  
  
  
  
  public void drive(double speed, double angle){
// Sets the motor to the speed passed in, will be the wheelSpeeds calculated in the SwerveDriveSubsystem	  
    speed_motor.set(speed);
// Calculates the setpoint measurement
    turnToAngle(angle);
// Applies the calculated pidOutput using the encoder as a measurementSource
  angle_motor.set(pidController.calculate(getModuleAngleAbsolute(), angle) * .2);
  }



  public void simpleDrive(double speed, double angle){
    speed_motor.set(speed);
    angle_motor.set(angle);
  }
  //public double getAnglePosition(){}

public void resetEncoder() {
  encoder.setPosition(0);
}

  public void simpleTurnToAngle(double targetAngle){
    angle_motor.set(pidController.calculate(getModuleAngle() * .2));
  }
}

