// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  PIDController pidController;
  WPI_TalonFX hoodMotor;
  /** Creates a new Hood. */
  public Hood(int hoodMotorID) {
    pidController = new PIDController(.025, 0, 0.002);
    pidController.setTolerance(100);
    pidController.setSetpoint(0);
    hoodMotor = new WPI_TalonFX(hoodMotorID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean getLimitSwitchState(){
    int hoodMotorState = hoodMotor.getSensorCollection().isRevLimitSwitchClosed();
    if(hoodMotorState == 0){
      return false;
    }
    else{
      return true;
    }
  }

  public PIDController getPIDController(){
    return pidController;
  }

  public void setHood(double speed){
    hoodMotor.set(speed);
  }

  public void setPosition(double targetPos){
    hoodMotor.set(ControlMode.Position, targetPos);
  }

public double getEncoderPosition() {
	return hoodMotor.getSelectedSensorPosition();
}
public void resetEncoder(){
  hoodMotor.setSelectedSensorPosition(0);

}


  
}
