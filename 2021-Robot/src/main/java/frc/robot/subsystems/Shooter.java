// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Shooter extends SubsystemBase {

  public Shooter(){

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


  public double getYOffset(){
    return NetworkTableInstance.getDefault().getTable("limelight-hounds").getEntry("ty").getDouble(0);
  }
}
