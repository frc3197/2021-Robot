// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.commands.Mechs.Continuous.shoot;

/** Add your docs here. */
public class Shooter extends SubsystemBase {
  private WPI_TalonFX shooter1,shooter2,shooter3;
  private static PIDController shooterPID = new PIDController(Constants.shooter_P, 0, Constants.shooter_D);
  private PIDController alignPID;
  private SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(Constants.shooter_kS, Constants.shooter_kV,
      Constants.shooter_kA);

  public Shooter(int shooter1, int shooter2, int shooter3) {
    this.shooter1 = new WPI_TalonFX(shooter1);
    this.shooter2 = new WPI_TalonFX(shooter2);
    this.shooter3 = new WPI_TalonFX(shooter3);

    alignPID = new PIDController(.0095, 0, 0.001);
    alignPID.setTolerance(2);

    this.shooter1.configOpenloopRamp(2);
    this.shooter2.configOpenloopRamp(2);
    this.shooter3.configOpenloopRamp(2);

    this.shooter1.setInverted(true);
    this.shooter2.setInverted(true);
    this.shooter3.setInverted(true);
    shooterPID.setTolerance(200, 200);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("VEL ERROR", shooterPID.getVelocityError());
    SmartDashboard.putNumber("PISS ERROR", shooterPID.getPositionError());
    // This method will be called once per scheduler run
  }

  public PIDController getAlignPIDController() {
    return alignPID;
  }

  public static PIDController getShooterPIDController() {
    return shooterPID;
  }

  public boolean hasTargets(){
    if(NetworkTableInstance.getDefault().getTable("limelight-hounds").getEntry("tv").getDouble(0) == 1){
      return true;
    }else{
      return false;
    }
  }

  public double getXOffset(){
    return NetworkTableInstance.getDefault().getTable("limelight-killroy").getEntry("tx").getDouble(0);
  }

  public double getRPM(){
    // Returns the Rate(Ticks Per 50MS)  * EncoderResolution / 60000
    return (shooter1.getSelectedSensorVelocity() * 2048) * 0.00166667 / 10;
  }

  public void setAllMotorsVoltage(double input){
    double PIDEffort = shooterPID.calculate(getRPM(), 6800);
    double FFEffort = shooterFF.calculate((getRPM() * 60));
    shooter1.setVoltage(input * (PIDEffort + FFEffort));
    shooter2.setVoltage(input * (PIDEffort + FFEffort));
    shooter3.setVoltage(input * (PIDEffort + FFEffort));
  }

  public double getYOffset(){
    return NetworkTableInstance.getDefault().getTable("limelight-killroy").getEntry("ty").getDouble(0);
  }
}
