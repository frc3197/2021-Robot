// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Shooter extends SubsystemBase {
  private WPI_TalonFX shooter1,shooter2,shooter3;

  public Shooter(int shooter1,int shooter2, int shooter3){
   this.shooter1 = new WPI_TalonFX(shooter1);
this.shooter2 = new WPI_TalonFX(shooter2);
    this.shooter3 = new WPI_TalonFX(shooter3);

  
this.shooter1.configOpenloopRamp(.5);
this.shooter2.configOpenloopRamp(.5);
    this.shooter3.configOpenloopRamp(.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean hasTargets(){
    if(NetworkTableInstance.getDefault().getTable("limelight-hounds").getEntry("tv").getDouble(0) == 1){
      return true;
    }else{
      return false;
    }
  }

  public double getXOffset(){
    return NetworkTableInstance.getDefault().getTable("limelight-hounds").getEntry("tx").getDouble(0);
  }

  public void setAllMotors(double speed){
    shooter1.set(-speed);
    shooter2.set(-speed);
    shooter3.set(-speed);
  }

  public double getYOffset(){
    return NetworkTableInstance.getDefault().getTable("limelight-hounds").getEntry("ty").getDouble(0);
  }
}
