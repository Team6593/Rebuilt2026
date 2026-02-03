// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

/** Add your docs here. */
public class IntakeInputs {
    public static final String kIntakeSpeedKey = "kIntakeSpeed";
    public static double kIntakeSpeed = .75;
    
    // PID
    public static final String kPivotPKey = "kPivotPKey";
    public static final String kPivotPositionKey = "kPivotPositionKey";
    public static double kPivotP = 50;
    public static double kPivotPosition = .3;
}
