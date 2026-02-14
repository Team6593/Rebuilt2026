// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.simulation.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSimulation extends SubsystemBase{

  private TalonFX shooter1 = new TalonFX(ShooterSimulationConstants.shooter1ID);
  private TalonFXSimState shooter1Sim = shooter1.getSimState();
  private final DCMotorSim m_motor1SimModel = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
      DCMotor.getKrakenX60(1), 0.06, ShooterSimulationConstants.shooter1Ratio),
    DCMotor.getKrakenX60(1));
  private TalonFXConfiguration shooter1Configs = new TalonFXConfiguration();

  private TalonFX shooter2 = new TalonFX(ShooterSimulationConstants.shooter2ID);
  private TalonFXSimState shooter2Sim = shooter2.getSimState();
  private final DCMotorSim m_motor2SimModel = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
      DCMotor.getKrakenX60(1), 0.06, ShooterSimulationConstants.shooter2Ratio),
    DCMotor.getKrakenX60(1));
  private TalonFXConfiguration shooter2Configs = new TalonFXConfiguration();

  private double distanceSim = 0;

  /** Creates a new ShooterSimulation. */
  public ShooterSimulation() {
    shooter1Sim.Orientation = ChassisReference.Clockwise_Positive;
    shooter1Sim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    shooter1Configs.Slot0.kP = ShooterSimulationInputs.shooterKP.get();
    shooter1Configs.Slot0.kV = ShooterSimulationInputs.shooterKV.get();
    shooter1Configs.Slot0.kA = ShooterSimulationInputs.shooterKA.get();
    shooter1Configs.Slot0.kS = ShooterSimulationInputs.shooterKS.get();
    shooter1Configs.CurrentLimits.StatorCurrentLimit = 80;
    shooter1Configs.CurrentLimits.StatorCurrentLimitEnable = true;
    shooter1Configs.CurrentLimits.SupplyCurrentLimit = 80;
    shooter1Configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooter1.getConfigurator().apply(shooter1Configs);
    
    shooter2Sim.Orientation = ChassisReference.Clockwise_Positive;
    shooter2Sim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    shooter2Configs.Slot0.kP = ShooterSimulationInputs.shooterKP.get();
    shooter2Configs.Slot0.kV = ShooterSimulationInputs.shooterKV.get();
    shooter2Configs.Slot0.kA = ShooterSimulationInputs.shooterKA.get();
    shooter2Configs.Slot0.kS = ShooterSimulationInputs.shooterKS.get();
    shooter2Configs.CurrentLimits.StatorCurrentLimit = 80;
    shooter2Configs.CurrentLimits.StatorCurrentLimitEnable = true;
    shooter2Configs.CurrentLimits.SupplyCurrentLimit = 80;
    shooter2Configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooter2.getConfigurator().apply(shooter2Configs);

    // shooter2.setControl(new Follower(ShooterSimulationConstants.shooter1ID, MotorAlignmentValue.Aligned));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var motor1Voltage = shooter1Sim.getMotorVoltageMeasure();
    m_motor1SimModel.setInputVoltage(motor1Voltage.in(Volts));
    m_motor1SimModel.update(0.020);
    shooter1Sim.setRawRotorPosition(m_motor1SimModel.getAngularPosition().times(ShooterSimulationConstants.shooter1Ratio));
    shooter1Sim.setRotorVelocity(m_motor1SimModel.getAngularVelocity().times(ShooterSimulationConstants.shooter1Ratio));

    var motor2Voltage = shooter2Sim.getMotorVoltageMeasure();
    m_motor2SimModel.setInputVoltage(motor2Voltage.in(Volts));
    m_motor2SimModel.update(0.020);
    shooter2Sim.setRawRotorPosition(m_motor2SimModel.getAngularPosition().times(ShooterSimulationConstants.shooter2Ratio));
    shooter2Sim.setRotorVelocity(m_motor2SimModel.getAngularVelocity().times(ShooterSimulationConstants.shooter2Ratio));

    sdLogging();
  }

  // Methods

  public void sdLogging() {
    SmartDashboard.putNumber("Shooter1Sim RPM", shooter1.getRotorVelocity().getValueAsDouble() * 60);
    SmartDashboard.putNumber("Shooter2Sim RPM", shooter2.getRotorVelocity().getValueAsDouble() * 60);
    SmartDashboard.putNumber("Shooter1Sim V", shooter1.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Shooter2Sim V", shooter2.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("ShooterSim Distance", distanceSim);
  }

  /**
   * Sets simulated shooter to specified speed.
   * @param RPM1 - RPM to run shooter1 at
   * @param RPM2 - RPM to run shooter2 at
   */
  public void setRPM(double RPM1, double RPM2) {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    shooter1.setControl(m_request.withVelocity(RPM1 / 60));
    final VelocityVoltage m_request2 = new VelocityVoltage(0).withSlot(0);
    shooter2.setControl(m_request2.withVelocity(RPM2 / 60));
  }

  public double getRPM() {
    return shooter1.getVelocity().getValueAsDouble() * 60;
  }

  public void shoot(double speed) {
    shooter1.set(speed);
  }

  public double getDistance() {
    return distanceSim;
  }

  public void setDistance(double distance) {
    distanceSim = distance;
  }

  public void stop() {
    shooter1.stopMotor();
    shooter2.stopMotor();
  }

  public Command runShooterRPM(double rpm1, double rpm2) {
    return this.runEnd(
      () -> setRPM(rpm1, rpm2), 
      () -> stop());
  }

  public Command stopShooterSimCommand() {
    return this.runOnce(
      () -> stop());
  }
  
}
