// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {

  private static final NetworkTable table =
    NetworkTableInstance.getDefault().getTable("limelight");

  /** Creates a new Limelight. */
  public Limelight() {
    LimelightHelpers.setPipelineIndex("base", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sdLogging();
  }

  public void sdLogging() {
    SmartDashboard.putNumber("Distance (in.)", estimateDistance());
  }

  public static double getDistanceToTagInches() {
    double[] botpose = table
      .getEntry("botpose_targetspace")
      .getDoubleArray(new double[6]);
    double x = botpose[0];
    double y = botpose[1];

    double distanceMeters = Math.sqrt(x * x + y * y);
    return Units.metersToInches(distanceMeters);
  }

  public double estimateDistance() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    return LimelightHelpers.estimateDistance(LimelightConstants.limelightMountAngleDegrees, LimelightConstants.limelightLensHeightInches, LimelightConstants.goalHeightInches, ty.getDouble(0));
  }

}
