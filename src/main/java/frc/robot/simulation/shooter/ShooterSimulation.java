// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation.shooter;

import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSimulation extends SubsystemBase implements ShooterSimulationConstants{

  private TalonFX shooter = new TalonFX(shooterID);
  private TalonFXSimState shooterSim = shooter.getSimState();
  private final DCMotorSim m_motorSimModel = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
      DCMotor.getKrakenX60(shooterID), 0.06, shooterRatio),
    DCMotor.getKrakenX60(shooterID));
  private TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();
  private CurrentLimitsConfigs shooterLimitsConfigs = new CurrentLimitsConfigs();

  /** Creates a new ShooterSimulation. */
  public ShooterSimulation() {
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
  
}
