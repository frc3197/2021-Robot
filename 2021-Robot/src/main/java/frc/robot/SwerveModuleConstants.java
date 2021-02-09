// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class SwerveModuleConstants {

    public final int AngleCANID,DriveCANID,EncoderCANID;

    public final double kS,kV,P,D;

    public SwerveModuleConstants(int AngleCANID, int DriveCANID, int EncoderCANID, double kS, double kV, double P, double D){
    //Motor CAN Constants
    this.AngleCANID = AngleCANID;
    this.DriveCANID = DriveCANID;
    //Encoder CAN Constants
    this.EncoderCANID = EncoderCANID;
    //FeedForward Constants
    this.kS = kS;
    this.kV = kV;
    //PID Constants
    this.P = P;
    this.D = D;
    }


}
