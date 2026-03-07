// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Preferences;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShotCalculator;
import frc.robot.utils.SubsystemInterface;

public class ShooterSubsystem extends SubsystemBase implements SubsystemInterface{

  // motors
  private TalonFX shooterMasterMotor = new TalonFX(ShooterConstants.shooterMasterID);
  private TalonFX shooterSecondaryMotor = new TalonFX(ShooterConstants.shooterSecondaryID);
  private TalonFX indexerMotor = new TalonFX(ShooterConstants.indexerID);

  // configurators
  private TalonFXConfigurator shooterMasterConfigurator = shooterMasterMotor.getConfigurator();
  private TalonFXConfigurator shooterSecondaryConfigurator = shooterSecondaryMotor.getConfigurator();
  private TalonFXConfigurator indexerConfigurator = indexerMotor.getConfigurator();
  
  // control configs
  private TalonFXConfiguration shooter1Configs = new TalonFXConfiguration();
  private TalonFXConfiguration shooter2Configs = new TalonFXConfiguration();
  private TalonFXConfiguration indexerConfigs = new TalonFXConfiguration();

  /** Creates a new Shooter. */
  public ShooterSubsystem() {
    
    // configs
    shooter1Configs.Slot0.kP = .1;
    shooter1Configs.Slot0.kA = ShooterInputs.kA;
    shooter1Configs.Slot0.kV = ShooterInputs.kV;
    shooter1Configs.CurrentLimits.StatorCurrentLimit = 40;
    shooter1Configs.CurrentLimits.StatorCurrentLimitEnable = true;
    shooter1Configs.CurrentLimits.SupplyCurrentLimit = 40;
    shooter1Configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterMasterConfigurator.apply(shooter1Configs);

    shooter2Configs.Slot0.kP = .01;
    shooter2Configs.Slot0.kA = ShooterInputs.kA;
    shooter2Configs.Slot0.kV = ShooterInputs.kV;
    shooter2Configs.CurrentLimits.StatorCurrentLimit = 40;
    shooter2Configs.CurrentLimits.StatorCurrentLimitEnable = true;
    shooter2Configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterSecondaryConfigurator.apply(shooter2Configs);

    indexerConfigs.Slot0.kP = 1;
    indexerConfigs.CurrentLimits.StatorCurrentLimit = 40;
    indexerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    indexerConfigs.CurrentLimits.SupplyCurrentLimit = 40;
    indexerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexerConfigurator.apply(indexerConfigs);

    // shooterSecondaryMotor.setControl(new Follower(ShooterConstants.shooterMasterID, MotorAlignmentValue.Opposed));

    // preferences
    Preferences.initDouble(ShooterInputs.kPKey, ShooterInputs.kP);
    Preferences.initDouble(ShooterInputs.kVKey, ShooterInputs.kV);
    Preferences.initDouble(ShooterInputs.kAKey, ShooterInputs.kA);
    Preferences.initDouble(ShooterInputs.kShooterFeedForwardKey, ShooterInputs.kS);
    Preferences.initDouble(ShooterInputs.kShooterSpeedKey, ShooterInputs.kShooterSpeed);
    Preferences.initDouble(ShooterInputs.kIndexerSpeedKey, ShooterInputs.kIndexerSpeed);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // loadPreferences();
    sdLogging();
  }

  // Data Logging

  /**
   * Values from the Shooter being published to SD.
   */
  @Override
  public void sdLogging() {
    SmartDashboard.putNumber("ShooterM Duty Cycle", shooterMasterMotor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("ShooterS Duty Cycle", shooterSecondaryMotor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("Indexer Duty Cycle", indexerMotor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("ShooterM Applied Output V", shooterMasterMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("ShooterS Applied Output V", shooterSecondaryMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Indexer Applied Output V", indexerMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("ShooterM RPS", shooterMasterMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("ShooterS RPS", shooterSecondaryMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Indexer RPS", indexerMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("ShooterM RPM", shooterMasterMotor.getRotorVelocity().getValueAsDouble() * 60);
    SmartDashboard.putNumber("ShooterS RPM", shooterSecondaryMotor.getRotorVelocity().getValueAsDouble() * 60);
    SmartDashboard.putNumber("Indexer RPM", indexerMotor.getRotorVelocity().getValueAsDouble() * 60);
    SmartDashboard.putNumber("ShooterM Temp (F)", ((shooterMasterMotor.getDeviceTemp().getValueAsDouble()) * 1.8) + 32);
    SmartDashboard.putNumber("ShooterS Temp (F)", ((shooterSecondaryMotor.getDeviceTemp().getValueAsDouble()) * 1.8) + 32);
    SmartDashboard.putNumber("Indexer Temp (F)", ((indexerMotor.getDeviceTemp().getValueAsDouble()) * 1.8) + 32);
  }

  /**
   * Logic for loading preference keys when they're changed, for the shooter.
   */
  @Override
  public void loadPreferences() {
    if (ShooterInputs.kP != Preferences.getDouble(ShooterInputs.kPKey, ShooterInputs.kP)) {
      System.out.println("Old kP: " + ShooterInputs.kP);
      ShooterInputs.kP = Preferences.getDouble(ShooterInputs.kPKey, ShooterInputs.kP);
      shooter1Configs.Slot0.kP = ShooterInputs.kP;
      System.out.println("New kP: " + ShooterInputs.kP);
    }
    if (ShooterInputs.kV != Preferences.getDouble(ShooterInputs.kVKey, ShooterInputs.kV)) {
      System.out.println("Old kV: " + ShooterInputs.kV);
      ShooterInputs.kV = Preferences.getDouble(ShooterInputs.kVKey, ShooterInputs.kV);
      shooter1Configs.Slot0.kV = ShooterInputs.kV;
      System.out.println("New kV: " + ShooterInputs.kV);
    }
    if (ShooterInputs.kA != Preferences.getDouble(ShooterInputs.kAKey, ShooterInputs.kA)) {
      System.out.println("Old kA: " + ShooterInputs.kA);
      ShooterInputs.kA = Preferences.getDouble(ShooterInputs.kAKey, ShooterInputs.kA);
      shooter1Configs.Slot0.kA = ShooterInputs.kA;
      System.out.println("New kA: " + ShooterInputs.kA);
    }
    if (ShooterInputs.kS != Preferences.getDouble(ShooterInputs.kShooterFeedForwardKey, ShooterInputs.kS)) {
      System.out.println("Old kShooterFeedForward: " + ShooterInputs.kS);
      ShooterInputs.kS = Preferences.getDouble(ShooterInputs.kShooterFeedForwardKey, ShooterInputs.kS);
      System.out.println("New kShooterFeedForward: " + ShooterInputs.kS);
    }
    if (ShooterInputs.kShooterSpeed != Preferences.getDouble(ShooterInputs.kShooterSpeedKey, ShooterInputs.kShooterSpeed)) {
      System.out.println("Old kShooterSpeed: " + ShooterInputs.kShooterSpeed);
      ShooterInputs.kShooterSpeed = Preferences.getDouble(ShooterInputs.kShooterSpeedKey, ShooterInputs.kShooterSpeed);
      System.out.println("New kShooterSpeed: " + ShooterInputs.kShooterSpeed);
    }
    if (ShooterInputs.kIndexerSpeed != Preferences.getDouble(ShooterInputs.kIndexerSpeedKey, ShooterInputs.kIndexerSpeed)) {
      System.out.println("Old kIndexerSpeed: " + ShooterInputs.kIndexerSpeed);
      ShooterInputs.kIndexerSpeed = Preferences.getDouble(ShooterInputs.kIndexerSpeedKey, ShooterInputs.kIndexerSpeed);
      System.out.println("New kIndexerSpeed: " + ShooterInputs.kIndexerSpeed);
    }
  }

  /**
   * just the shoot (top/main/flywheel)
   * @param speed to move the shoot motor at - Defaults to value in {@link ShooterInputs}
   */
  public void shoot(double speed) {
    shooterMasterMotor.set(speed);
  }

    /**
   * just the shoot (top/main/flywheel)
   * @param speed to move the shoot motor at - Defaults to value in {@link ShooterInputs}
   */
  public void shoot() {
    shooterMasterMotor.set(ShooterInputs.kShooterSpeed);
  }

  /**
   * just the throughput (bottom/secondary/torque)
   * @param speed to move the shoot motor at - Defaults to value in {@link ShooterInputs}
   */
  public void index(double speed) {
    indexerMotor.set(speed);
  }

    /**
   * just the throughput (bottom/secondary/torque)
   * @param speed to move the shoot motor at - Defaults to value in {@link ShooterInputs}
   */
  public void index() {
    indexerMotor.set(ShooterInputs.kIndexerSpeed);
  }

  public void indexAndShoot(double shooterSpeed, double indexerSpeed) {
    shooterMasterMotor.set(shooterSpeed);
    indexerMotor.set(indexerSpeed);
  }

  /**
   * Stops all motors (shooter, indexer) in the shooter subsystem.
   */
  @Override
  public void stop() {
    shooterMasterMotor.stopMotor();
    shooterSecondaryMotor.stopMotor();
    indexerMotor.stopMotor();
  }

  public void stopIndexer() {
    indexerMotor.stopMotor();
  }

  public void stopShooter() {
    shooterMasterMotor.stopMotor();
  }

  /**
   * Method that sets the RPM.
   * @param RPM - Desired RPM (Do not put RPS, method divides by 60) (default in {@link ShooterConstants})
   * @param feedForward - Desired feedforward (V to overcome gravity) (default in {@link ShooterConstants})
   */
  public void setMasterRPM(double RPM1, double RPM2) {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    shooterMasterMotor.setControl(m_request.withVelocity(RPM1 / 60));
    final VelocityVoltage m_request2 = new VelocityVoltage(0).withSlot(0);
    shooterSecondaryMotor.setControl(m_request2.withVelocity(RPM2 / 60));
  }

  public void setIndexerRPM(double RPM) {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    indexerMotor.setControl(m_request.withVelocity(RPM / 60));
  }

  public double getShooterRPM() {
    return shooterMasterMotor.getRotorVelocity().getValueAsDouble() * 60;
  }

  public double getIndexerRPM() {
    return indexerMotor.getRotorVelocity().getValueAsDouble() * 60;
  }

  public double getDutyCycle() {
    return shooterMasterMotor.getDutyCycle().getValueAsDouble();
  }


  /**
   * Method that sets the RPM.
   * @param RPM - Desired RPM (Do not put RPS, method divides by 60) (default in {@link ShooterConstants}
   */
  public void setShooterRPM(double RPM) {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    shooterMasterMotor.setControl(m_request.withVelocity(RPM / 60));
  }

  /**
   * Method that sets the RPM.
   * @param RPM - Desired RPM (Do not put RPS, method divides by 60) (default in {@link ShooterConstants}
   */
  public void setShooterRPM() {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    shooterMasterMotor.setControl(m_request.withVelocity(ShooterInputs.kShooterGoalRPS));
  }

  public void setIndexerRPM() {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    indexerMotor.setControl(m_request.withVelocity(ShooterInputs.kShooterGoalRPS));
  }

  // Commands

  /**
   * Command that stops the shooter.
   * @return command
   */
  public Command stopCommand() {
    return this.runOnce(
      () -> stop());
  }

  /**
   * Command that runs the shooter and indexer.
   * @param shooterSpeed - Desired duty cycle (default in {@link ShooterInputs})
   * @param indexerSpeed - Desired duty cycle (default in {@link ShooterInputs})
   * @return command
   */
  public Command shootCommand(double shooterSpeed, double indexerSpeed) {
    return this.run(
      () -> indexAndShoot(shooterSpeed, indexerSpeed)).andThen(stopCommand());
  }

  /**
   * Just runs the shooter.
   * @param shooterSpeed - Defaults in {@link ShooterInputs}
   * @return
   */
  public Command justShootCommand(double shooterSpeed) {
    return this.runEnd(
      () -> shoot(shooterSpeed),
      () -> stop()
    );
  }

  /**
   * Just runs the shooter.
   * @param shooterSpeed - Defaults in {@link ShooterInputs}
   * @return
   */
  public Command justShootCommand() {
    return this.runEnd(
      () -> shoot(),
      () -> stop()
    );
  }

  /**
   * Just runs the indexer.
   * @param indexSpeed - Defaults in {@link ShooterInputs}
   * @return
   */
  public Command justIndexCommand(double indexSpeed) {
    return this.runEnd(
      () -> index(indexSpeed), 
      () -> stop()
    );
  }

  /**
   * Just runs the indexer.
   * @param indexSpeed - Defaults in {@link ShooterInputs}
   * @return
   */
  public Command justIndexCommand() {
    return this.runEnd(
      () -> index(), 
      () -> stop()
    );
  }

  /**
   * Command that runs the shooter and indexer.
   * @param shooterSpeed - Desired duty cycle (default in {@link ShooterConstants})
   * @param indexerSpeed - Desired duty cycle (default in {@link ShooterConstants})
   * @return command
   */
  public Command shootCommand(double shooterSpeed) {
    return this.run(
      () -> indexAndShoot(shooterSpeed, ShooterInputs.kIndexerSpeed)).andThen(stopCommand());
  }

  /**
   * Command that runs the shooter and indexer.
   * @param shooterSpeed - Desired duty cycle (default in {@link ShooterConstants})
   * @param indexerSpeed - Desired duty cycle (default in {@link ShooterConstants})
   * @return command
   */
  public Command shootCommand() {
    return this.run(
      () -> indexAndShoot(ShooterInputs.kShooterSpeed, ShooterInputs.kIndexerSpeed)).andThen(stopCommand());
  }

  /**
   * Comamnd that sets the RPM.
   * @param RPM - Desired RPM (Do not put RPS, method divides by 60) (default in {@link ShooterConstants.java})
   * @return command
   */
  public Command setRPMCommand() {
    return this.run(
      () -> setShooterRPM()).andThen(stopCommand());
  }

}
