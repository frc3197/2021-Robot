// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.photonvision.PhotonCamera;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



public class Intake extends SubsystemBase {

  CANSparkMax intakeMotor;
  
  static PhotonCamera camera = new PhotonCamera("intakeCam");
  
  /** Creates a new Intake. */
  public Intake(int intakeID) {
    intakeMotor = new CANSparkMax(intakeID,MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static PhotonCamera getCam(){
    return camera;
  }

  public void setIntake(double speed){

    intakeMotor.set(speed);

  }

  public double getYaw(){
    var result = camera.getLatestResult();
    double output = 0;
    if(result.hasTargets()){
      output = result.getBestTarget().getYaw();
    }
    return output;
  }
}
