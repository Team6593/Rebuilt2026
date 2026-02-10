// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.simulation.shooter;

import static edu.wpi.first.units.Units.Volts;

import java.util.TreeMap;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShooterParams;

public class ShooterSimulation extends SubsystemBase implements ShooterSimulationConstants  {

  private TalonFX shooter = new TalonFX(shooterID);
  private TalonFXSimState shooterSim = shooter.getSimState();
  private final DCMotorSim m_motorSimModel = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
      DCMotor.getKrakenX60(shooterID), 0.06, shooterRatio),
    DCMotor.getKrakenX60(shooterID));
  private TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();
  private CurrentLimitsConfigs shooterLimitsConfigs = new CurrentLimitsConfigs();
  private static final TreeMap<Double, ShooterParams> shooterMap = new TreeMap<>();
  static {
    shooterMap.put(1.0, new ShooterParams(1000, .5));
    shooterMap.put(2.0, new ShooterParams(1200, .6));
    shooterMap.put(3.0, new ShooterParams(1400, .7));
    shooterMap.put(4.0, new ShooterParams(1600, .8));
    shooterMap.put(5.0, new ShooterParams(2000, 1.0));
    shooterMap.put(6.0, new ShooterParams(2100, 1.4));    
  }

  /** Creates a new ShooterSimulation. */
  public ShooterSimulation() {
    System.out.println("Initializing Tunables: " + ShooterSimulationInputs.kPosition.get());
    shooterSim.Orientation = ChassisReference.Clockwise_Positive;
    shooterSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    shooterConfigs.MotionMagic.MotionMagicCruiseVelocity = 6000;
    shooterConfigs.Slot0.kP = ShooterSimulationInputs.shooterKP.get();
    shooterConfigs.Slot0.kV = ShooterSimulationInputs.shooterKV.get();
    shooterConfigs.Slot0.kA = ShooterSimulationInputs.shooterKA.get();
    shooterConfigs.Slot0.kS = ShooterSimulationInputs.shooterKS.get();
    shooterLimitsConfigs.StatorCurrentLimit = 80;
    shooterLimitsConfigs.SupplyCurrentLimit = 80;
    shooterLimitsConfigs.StatorCurrentLimitEnable = true;
    shooterLimitsConfigs.SupplyCurrentLimitEnable = true;
    shooter.getConfigurator().apply(shooterLimitsConfigs);
    shooter.getConfigurator().apply(shooterConfigs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    var motorVoltage = shooterSim.getMotorVoltageMeasure();
    m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
    m_motorSimModel.update(0.020);
    shooterSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(shooterRatio));
    shooterSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(shooterRatio));
    smartdashboardLogging();
  }

  // Methods

  public void smartdashboardLogging() {
    SmartDashboard.putNumber("ShooterSim RPM", shooter.getRotorVelocity().getValueAsDouble() * 60);
  }

  /**
   * Gets interpolated RPM.
   * @param key - current distance
   * @return
   */
  public ShooterParams getInterpolatedRPM(double key) {
    if (shooterMap.containsKey(key)) {
      return shooterMap.get(key);
    }
    Double lowerKey = shooterMap.floorKey(key);
    Double upperKey = shooterMap.ceilingKey(key);
    if (lowerKey == null) {
      return shooterMap.get(upperKey);
    }
    if (upperKey == null) {
      return shooterMap.get(lowerKey);
    }
    ShooterParams lowerParams = shooterMap.get(lowerKey);
    ShooterParams upperParams = shooterMap.get(upperKey);
    double ratio = (key - lowerKey) / (upperKey - lowerKey);
    double rpm = interpolate(lowerParams.rpm, upperParams.rpm, ratio);
    double tof = interpolate(lowerParams.tof, upperParams.tof, ratio);

    return new ShooterParams(rpm, tof);
  }

  /**
   * Interpolation method for the tree map.
   * @param start - lower params
   * @param end - upper params
   * @param ratio - ratio
   * @return
   */
  private static double interpolate(double start, double end, double ratio) {
    return start + (end - start) * ratio;
  }

  /**
   * Sets simulated shooter to specified speed.
   * @param speed - Defaults to value in ShooterSimulationInputs.java
   */
  public void setRPM(double RPM) {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    shooter.setControl(m_request.withVelocity(RPM / 60));
  }

  /**
   * Sets simulated shooter to specified speed.
   * @param speed - Defaults to value in ShooterSimulationInputs.java
   */
  public void setRPM() {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    shooter.setControl(m_request.withVelocity(ShooterSimulationInputs.shooterRPM.get() / 60));
  }

  public double getRPM() {
    return shooter.getVelocity().getValueAsDouble() * 60;
  }

  public void shoot(double speed) {
    shooter.set(speed);
  }

  public void stop() {
    shooter.stopMotor();
  }

  // Commands
  /**
   * Command that sets the rpm.
   * @param RPM
   * @return - command
   */
  public Command setRPMCommand(double RPM) {
    return this.runEnd(
      () -> setRPM(RPM),
      () -> stop());
  }

  /**
   * Command that sets the rpm.
   * @return - command
   */
  public Command setRPMCommand() {
    return this.runEnd(
      () -> setRPM(),
      () -> stop());
  }

  public Command stopRPMCommand() {
    return this.runOnce(
      () -> stop());
  }
  
}
