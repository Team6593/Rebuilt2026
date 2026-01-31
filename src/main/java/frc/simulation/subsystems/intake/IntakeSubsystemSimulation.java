// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.simulation.subsystems.intake;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystemSimulation extends SubsystemBase implements IntakeConsantsSimulation {

  private SparkMax pivot = new SparkMax(pivotID, MotorType.kBrushless);
  private SparkClosedLoopController pivotController = pivot.getClosedLoopController();
  private SparkMaxConfig pivotConfig = new SparkMaxConfig();
  private DCMotor dcGearbox = DCMotor.getNeoVortex(1);
  private SparkMaxSim pivotSim = new SparkMaxSim(pivot, dcGearbox);

  /** Creates a new IntakeSimulationSubsystem. */
  @SuppressWarnings("removal")
  public IntakeSubsystemSimulation() {
    pivotConfig.closedLoop
      .p(IntakeInputsSimulation.pivotKP.get());
    pivotConfig.closedLoop.feedForward
      .kV(IntakeInputsSimulation.pivotKV.get());
    pivotConfig.closedLoop.maxMotion
      .cruiseVelocity(IntakeInputsSimulation.pivotCruiseVelocity.get())
      .maxAcceleration(IntakeInputsSimulation.pivotMaxAcceleration.get())
      .allowedProfileError(IntakeInputsSimulation.pivotAllowedProfileError.get());
    pivot.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sdLogging();
  }
  
  private void sdLogging() {
    SmartDashboard.putNumber("PivotSim Position", pivotSim.getPosition());
    SmartDashboard.putNumber("PivotSim Applied Output", pivotSim.getAppliedOutput());
    SmartDashboard.putNumber("PivotSim Motor Current (A)", pivotSim.getMotorCurrent());
    SmartDashboard.putBoolean("PivotSim Setpoint", pivotController.isAtSetpoint());
  }

  /**
   * Pivots to setpoint.
   * @param setpoint
   */
  public void pivotToSetpoint(double setpoint) {
    pivotController.setSetpoint(setpoint, ControlType.kMAXMotionPositionControl);
  }

  public void stop() {
    pivot.stopMotor();
  }

  public boolean isAtSetpoint() {
    return pivotController.isAtSetpoint();
  }
}
