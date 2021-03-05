// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Agitator extends SubsystemBase {
  /** Creates a new Agitator. */
  CANSparkMax agitatorMotor;
  public Agitator(int agitatorCANID) {
    agitatorMotor = new CANSparkMax(agitatorCANID,MotorType.kBrushless);
    agitatorMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setAgitatorMotor(double speed){
    agitatorMotor.set(speed);
  }
}
