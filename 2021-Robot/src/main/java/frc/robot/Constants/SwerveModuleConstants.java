// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

/** Add your docs here. */
public class SwerveModuleConstants {

    public final int AngleCANID,DriveCANID,EncoderCANID;

    public final double kS_angle,kV_angle,P_angle,D_angle;

    public final double kS_drive,kV_drive,P_drive,D_drive;

    public SwerveModuleConstants(int AngleCANID, int DriveCANID, int EncoderCANID, double kS_angle, double kV_angle, double P_angle, double D_angle,double kS_drive, double kV_drive, double P_drive, double D_drive){
    //Motor CAN Constants
    this.AngleCANID = AngleCANID;
    this.DriveCANID = DriveCANID;
    //Encoder CAN Constants
    this.EncoderCANID = EncoderCANID;
    //FeedForward Constants
    this.kS_angle = kS_angle;
    this.kS_drive = kS_drive;
    this.kV_angle = kV_angle;
    this.kV_drive = kV_drive;
    //PID Constants
    this.P_angle = P_angle;
    this.P_drive = P_drive;
    this.D_angle = D_angle;
    this.D_drive = D_drive;
    }


}
