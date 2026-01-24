// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class ShooterParams {

    public double rpm;
    public double tof;

    /**
     * ShooterParams class for map, stores RPM and TOF (time of flight) pairs.
     * @param rpm
     * @param tof
     */
    public ShooterParams(double rpm, double tof) {
        this.rpm = rpm;
        this.tof = tof;
    }

}
