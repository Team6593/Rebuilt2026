// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.ShotCalculator;

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

  // Methods

  /**
   * Logs values onto SD.
   */
  public void sdLogging() {
    SmartDashboard.putNumber("Distance (in.)", estimateDistance());
    SmartDashboard.putNumber("Target RPM", ShotCalculator.lerpGet(estimateDistance()).rpm);
  }

  /**
   * Gets distance from tag to inches using 3d apriltags (dont think it works?)
   * @return distance (inches)
   */
  public static double getDistanceToTagInches() {
    double[] botpose = table
      .getEntry("botpose_targetspace")
      .getDoubleArray(new double[6]);
    double x = botpose[0];
    double y = botpose[1];

    double distanceMeters = Math.sqrt(x * x + y * y);
    return Units.metersToInches(distanceMeters);
  }

  /**
   * Just steals from limelighthelpers lol
   * @return distance (inches)
   */
  public double estimateDistance() {
    if (hasValidTargets()) {
      return LimelightHelpers.getTargetPose_RobotSpace("limelight")[2] * 39.37;
    } else {
      return 0;
    }
  }


  public double limelightAimProportional() {
    double kP = LimelightConstants.kAimP;
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
    targetingAngularVelocity *= 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    targetingAngularVelocity *= -1.0;
    return targetingAngularVelocity;
  }
  
  public boolean hasValidTargets() {
    if (LimelightHelpers.getFiducialID("limelight") > 0) {
      return true;
    } else {
      return false;
    }
  }

  public double getID() {
    return LimelightHelpers.getFiducialID("limelight");
  }

}
