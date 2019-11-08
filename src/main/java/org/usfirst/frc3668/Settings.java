/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc3668;

/**
 * Add your docs here.
 */
public class Settings {
    public static final int ticsPerRev = 4096;
    public static final double chassisEncoderDistancePerPulse = (6 * Math.PI) / ticsPerRev;
    public static final double chassisMaxInchesPerSecond = 168;
    public static final double driveKp = 0.1;
    public static final double driveKi = 0.002;
    public static final double driveKd = 0.0009;
    public static final double profileMovementThreshold = 0.75;
    public static final double chassisDriveStraightGyroKp = 0.05;
}
