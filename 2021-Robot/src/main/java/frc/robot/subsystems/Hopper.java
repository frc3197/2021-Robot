// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
  CANSparkMax hopperMotor;
  public static int count = 0;

  /** Creates a new Hopper. */
  public Hopper(int hopperCANID) {
    hopperMotor = new CANSparkMax(hopperCANID);

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
