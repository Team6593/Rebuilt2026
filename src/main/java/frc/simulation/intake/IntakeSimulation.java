// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.simulation.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import org.opencv.core.Mat;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSimulation extends SubsystemBase {

  private SparkMax motor1 = new SparkMax(23, MotorType.kBrushless);
  private SparkMaxSim motor1Sim = new SparkMaxSim(motor1, DCMotor.getNEO(1));
  private SparkMax motor2 = new SparkMax(24, MotorType.kBrushless);
  private SparkMaxSim motor2Sim = new SparkMaxSim(motor2, DCMotor.getNEO(1));
  private SparkMaxConfig motor2Config = new SparkMaxConfig();
  private DCMotorSim motor1SimModel = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
      DCMotor.getNEO(1), 0.06, 9), 
      DCMotor.getNEO(1));
  private DCMotorSim motor2SimModel = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(
      DCMotor.getNEO(1), 0.06, 9), 
      DCMotor.getNEO(1));
  private ProfiledPIDController pidController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(1000, 1000));

  /** Creates a new IntakeSimulation. */
  public IntakeSimulation() {
    stop();
    motor2Config.follow(motor1.getDeviceId());
    motor2.configure(motor2Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Intake Simulation/Mechanism Pos", motor1Sim.getPosition());
    SmartDashboard.putNumber("Intake Simulation/Motor1 V", motor1SimModel.getInputVoltage());
    SmartDashboard.putNumber("Intake Simulation/Motor1 RPM", motor1Sim.getVelocity());
    SmartDashboard.putNumber("Intake Simulation/Motor2 V", motor2SimModel.getInputVoltage());
    SmartDashboard.putNumber("Intake Simulation/Motor2 RPM", motor2Sim.getVelocity());
    SmartDashboard.putBoolean("Intake Simulation/At Setpoint", atSetpoint());
  }

  @Override
  public void simulationPeriodic() {

    double conversionFactor = 360;
    var motor1Voltage = motor1.getAppliedOutput() * motor1Sim.getBusVoltage();
    motor1SimModel.setInputVoltage(motor1Voltage);
    motor1SimModel.update(0.020);
    motor1Sim.setPosition(Units.radiansToRotations(motor1SimModel.getAngularPositionRad()) * conversionFactor);
    motor1Sim.iterate(Units.radiansPerSecondToRotationsPerMinute(motor1SimModel.getAngularVelocityRadPerSec()) * conversionFactor, motor1.getBusVoltage(), 0.020);
    motor2SimModel.setInputVoltage(motor1Voltage);
    motor2SimModel.update(0.020);
    motor2Sim.setPosition(Units.radiansToRotations(motor2SimModel.getAngularPositionRad()) * conversionFactor);
    motor2Sim.iterate(Units.radiansPerSecondToRotationsPerMinute(motor2SimModel.getAngularVelocityRadPerSec()) * conversionFactor, motor2.getBusVoltage(), 0.020);
  }

  public void pidToSetpoint(double setpoint, double kP) {
    pidController.setTolerance(1);
    pidController.setP(kP);
    pidController.setGoal(setpoint);
    var pidOutput = 
      pidController.calculate(
        motor1Sim.getPosition(), setpoint);
    double clampedVoltage = Math.max(-12.0, Math.min(12.0, pidOutput));
    System.out.println("Goal: " + pidController.getGoal().position + " | Curr: " + motor1Sim.getPosition() + " | Out: " + clampedVoltage);
    motor1.setVoltage(clampedVoltage);
  }

  public void stop() {
    motor1.stopMotor();
  }

  public boolean atSetpoint() {
    return pidController.atGoal();
  }

  public Command pidToSetpointCommand(double setpoint, double kP) {
    return this.run(
      () -> pidToSetpoint(setpoint, kP));
  }

  public Command stopCommand() {
    return this.runOnce(
      () -> stop());
  }

}
