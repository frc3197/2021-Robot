// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class ShooterLookupTable {

    public static int lookupEncoderTarget(int distance) {

         if (RobotContainer.isBetween(distance, 75)) {
            return 14000;
        } else if (RobotContainer.isBetween(distance, 85)) {
            return 15000;
        } else if (RobotContainer.isBetween(distance, 95)) {
            return 18000;
        } else if (RobotContainer.isBetween(distance, 105)) {
            return 20000;
        } else if (RobotContainer.isBetween(distance, 115)) {
            return 21000;
        } else if (RobotContainer.isBetween(distance, 125)) {
            return 22000;
        } else if (RobotContainer.isBetween(distance, 135)) {
            return 25000;
        } else if (RobotContainer.isBetween(distance, 145)) {
            return 25000;
        } else if (RobotContainer.isBetween(distance, 155)) {
            return 26000;
        } else if (RobotContainer.isBetween(distance, 165)) {
            return 28000;
        } else if (RobotContainer.isBetween(distance, 175)) {
            return 26000;
        } else if (RobotContainer.isBetween(distance, 185)) {
            return 28000;
        } else if (RobotContainer.isBetween(distance, 195)) {
            return 28700;
        } else if (RobotContainer.isBetween(distance, 205)) {
            return 30000;
        } else if (RobotContainer.isBetween(distance, 215)) {
            return 27000;
        } else if (RobotContainer.isBetween(distance, 225)) {
            return 27400;
        } else if (RobotContainer.isBetween(distance, 235)) {
            return 27500;
        } else if (RobotContainer.isBetween(distance, 245)) {
            return 26500;
        } else if (RobotContainer.isBetween(distance, 255)) {
            return 26300;
        } else if (RobotContainer.isBetween(distance, 265)) {
            return 29000;
        } else if (RobotContainer.isBetween(distance, 275)) {
            return 28000;
        }

        else {
            return 14000;
        }

    }
}
