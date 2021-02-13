// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANSparkMaxID;

public class Hopper extends SubsystemBase {
  WPI_TalonFX hopperMotor;
  CANSparkMax agitatorMotor;
  public static int count = 0;

  /** Creates a new Hopper. */
  public Hopper(int lifterCANID, int agitatorCANID) {
    hopperMotor = new WPI_TalonFX(lifterCANID);
    agitatorMotor = new CANSparkMax(agitatorCANID, MotorType.kBrushless);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /** Creates a new Hopper. */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void sethopperMotor(double speed) {
    hopperMotor.set(speed);
  }

}
