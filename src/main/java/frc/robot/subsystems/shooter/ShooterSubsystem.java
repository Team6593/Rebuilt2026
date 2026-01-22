// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  // motors
  private TalonFX shooterMotor = new TalonFX(ShooterConstants.shooterID);
  private TalonFX indexerMotor = new TalonFX(ShooterConstants.indexerID);

  // configurators
  private TalonFXConfigurator shootConfigurator = shooterMotor.getConfigurator();
  private TalonFXConfigurator indexerConfigurator = indexerMotor.getConfigurator();

  // current limit configs
  private CurrentLimitsConfigs shooterLimitConfigs = new CurrentLimitsConfigs();
  private CurrentLimitsConfigs indexerLimitConfigs = new CurrentLimitsConfigs();
  
  // control configs
  private Slot0Configs shooterConfigs = new Slot0Configs();

  /** Creates a new Shooter. */
  public ShooterSubsystem() {
    
    shooterLimitConfigs.StatorCurrentLimit = 80;
    shooterLimitConfigs.StatorCurrentLimitEnable = true;
    indexerLimitConfigs.SupplyCurrentLimit = 60;
    indexerLimitConfigs.SupplyCurrentLimitEnable = true;

    shootConfigurator.apply(shooterLimitConfigs);
    indexerConfigurator.apply(indexerLimitConfigs);

    shooterConfigs.kP = ShooterInputs.kP;
    shooterConfigs.kV = ShooterInputs.kV;
    shooterConfigs.kA = ShooterInputs.kA; 

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Shooter Applied Output V", shooterMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Indexer Applied Output V", indexerMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Shooter RPS", shooterMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Indexer RPS", indexerMotor.getVelocity().getValueAsDouble());

  }

  // Methods

  /**
   * just the shoot (top/main/flywheel)
   * @param speed to move the shoot motor at
   */
  public void shoot(double speed) {
    shooterMotor.set(speed);
  }

  /**
   * just the throughput (bottom/secondary/torque)
   * @param speed to move the shoot motor at
   */
  public void index(double speed) {
    indexerMotor.set(speed);
  }

  public void indexAndShoot(double shooterSpeed, double indexerSpeed) {
    shooterMotor.set(shooterSpeed);
    indexerMotor.set(indexerSpeed);
  }

  /**
   * Stops all motors (shooter, indexer) in the shooter subsystem.
   */
  public void stop() {
    shooterMotor.stopMotor();
    indexerMotor.stopMotor();
  }

  /**
   * Method that sets the RPM.
   * @param RPM - Desired RPM (Do not put RPS, method divides by 60) (default in ShooterConstants.java)
   * @param feedForward - Desired feedforward (V to overcome gravity) (default in ShooterConstants.java)
   */
  public void setRPM(double RPM, double feedForward) {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    shooterMotor.setControl(m_request.withVelocity(RPM / 60).withFeedForward(feedForward));
  }

  /**
   * Method that sets the RPM.
   * @param RPM - Desired RPM (Do not put RPS, method divides by 60) (default in ShooterConstants.java)
   * @param feedForward - Desired feedforward (V to overcome gravity) (default in ShooterConstants.java)
   */
  public void setRPM(double RPM) {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    shooterMotor.setControl(m_request.withVelocity(RPM / 60).withFeedForward(.5));
  }

  /**
   * Method that sets the RPM.
   * @param RPM - Desired RPM (Do not put RPS, method divides by 60) (default in ShooterConstants.java)
   * @param feedForward - Desired feedforward (V to overcome gravity) (default in ShooterConstants.java)
   */
  public void setRPM() {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    shooterMotor.setControl(m_request.withVelocity(ShooterInputs.kShooterGoalRPS).withFeedForward(ShooterInputs.kShooterFeedForward));
  }

  // Commands

  /**
   * Command that stops the shooter.
   * @return command
   */
  public Command stopCommand() {
    return this.run(
      () -> stop());
  }

  /**
   * Command that runs the shooter and indexer.
   * @param shooterSpeed - Desired duty cycle (default in ShooterConstants.java)
   * @param indexerSpeed - Desired duty cycle (default in ShooterConstants.java)
   * @return command
   */
  public Command shootCommand(double shooterSpeed, double indexerSpeed) {
    return this.run(
      () -> indexAndShoot(shooterSpeed, indexerSpeed)).andThen(stopCommand());
  }

  /**
   * Command that runs the shooter and indexer.
   * @param shooterSpeed - Desired duty cycle (default in ShooterConstants.java)
   * @param indexerSpeed - Desired duty cycle (default in ShooterConstants.java)
   * @return command
   */
  public Command shootCommand(double shooterSpeed) {
    return this.run(
      () -> indexAndShoot(shooterSpeed, ShooterInputs.kIndexerSpeed)).andThen(stopCommand());
  }

  /**
   * Command that runs the shooter and indexer.
   * @param shooterSpeed - Desired duty cycle (default in ShooterConstants.java)
   * @param indexerSpeed - Desired duty cycle (default in ShooterConstants.java)
   * @return command
   */
  public Command shootCommand() {
    return this.run(
      () -> indexAndShoot(ShooterInputs.kShooterSpeed, ShooterInputs.kIndexerSpeed)).andThen(stopCommand());
  }
  
  /**
   * Comamnd that sets the RPM.
   * @param RPM - Desired RPM (Do not put RPS, method divides by 60) (default in ShooterConstants.java)
   * @param feedForward - Desired feedforward (V to overcome gravity) (default in ShooterConstants.java)
   * @return command
   */
  public Command setRPMCommand(double RPM, double feedForward) {
    return this.run(
      () -> setRPM(RPM, feedForward)).andThen(stopCommand());
  }

  /**
   * Comamnd that sets the RPM.
   * @param RPM - Desired RPM (Do not put RPS, method divides by 60) (default in ShooterConstants.java)
   * @param feedForward - Desired feedforward (V to overcome gravity) (default in ShooterConstants.java)
   * @return command
   */
  public Command setRPMCommand(double RPM) {
    return this.run(
      () -> setRPM(RPM)).andThen(stopCommand());
  }

  /**
   * Comamnd that sets the RPM.
   * @param RPM - Desired RPM (Do not put RPS, method divides by 60) (default in ShooterConstants.java)
   * @param feedForward - Desired feedforward (V to overcome gravity) (default in ShooterConstants.java)
   * @return command
   */
  public Command setRPMCommand() {
    return this.run(
      () -> setRPM()).andThen(stopCommand());
  }

}
