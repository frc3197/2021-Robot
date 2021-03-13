// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.wpilibj.util.Units;
import frc.robot.subsystems.Drivetrain.SwerveDrive;

/** Add your docs here. */
public class AutoConstants {
    public final static double maxSpeed = Units.feetToMeters(1);
    public final static double maxAcceleration = Units.feetToMeters(.5);



    public static enum thetaPIDConstants{
        kP(.45),kI(0),kD(0.006);


        public final double constant;

        private thetaPIDConstants(double constant){
            this.constant = constant;
        }
    }

    public static enum xPIDConstants{
        kP(1),kI(0),kD(0);


        public final double constant;

        private xPIDConstants(double constant){
            this.constant = constant;
        }
    }

    public static enum yPIDConstants{
        kP(1),kI(0),kD(0);


        public final double constant;

        private yPIDConstants(double constant){
            this.constant = constant;
        }
    }




}
