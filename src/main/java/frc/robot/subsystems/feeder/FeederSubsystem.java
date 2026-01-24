// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase implements FeederConstants{

  // devices
  private TalonFX feederMotor = new TalonFX(feederMotorID);

  // current limit configs
  private CurrentLimitsConfigs feederCurrentLimitsConfigs = new CurrentLimitsConfigs();

  // configurator
  private TalonFXConfigurator feederConfigurator = feederMotor.getConfigurator();

  /** Creates a new ThroughputSubsytem. */
  public FeederSubsystem() {
    feederCurrentLimitsConfigs.StatorCurrentLimit = 40;
    feederCurrentLimitsConfigs.StatorCurrentLimitEnable = true;
    feederCurrentLimitsConfigs.SupplyCurrentLimit = 40;
    feederCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
    feederConfigurator.apply(feederCurrentLimitsConfigs);

    Preferences.initDouble(FeederInputs.kFeederSpeedKey, FeederInputs.kFeederSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    smartdashboardLogging();
    loadPreferences();
  }


  // Methods

  public void smartdashboardLogging() {
    SmartDashboard.putNumber("Feeder Motor Duty Cycle", feederMotor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("Feeder Motor Applied Output V", feederMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Feeder Motor RPM", feederMotor.getVelocity().getValueAsDouble() * 60);
    SmartDashboard.putNumber("Feeder Motor Temp (F)", ((feederMotor.getDeviceTemp().getValueAsDouble()) * 1.8) + 32);
  }

  /**
   * Loads the new preferences after changing them (ascope tuning).
   */
  public void loadPreferences() {
    if (FeederInputs.kFeederSpeed != Preferences.getDouble(FeederInputs.kFeederSpeedKey, FeederInputs.kFeederSpeed)) {
      System.out.println("Old kFeederSpeed: " + FeederInputs.kFeederSpeed);
      FeederInputs.kFeederSpeed = Preferences.getDouble(FeederInputs.kFeederSpeedKey, FeederInputs.kFeederSpeed);
      System.out.println("New kFeederSpeed: " + FeederInputs.kFeederSpeed);
    }
  }

  /**
   * Runs the feeder motor to feed fuel to the robot.
   * @param speed - Defaults to value in FeederInputs.java
   */
  public void feed(double speed) {
    feederMotor.set(speed);
  }

  /**
   * Runs the feeder motor to feed fuel to the robot.
   * @param speed - Defaults to value in FeederInputs.java
   */
  public void feed() {
    feederMotor.set(FeederInputs.kFeederSpeed);
  }

  public void stop() {
    feederMotor.stopMotor();
  }

  // Commands
  
  /**
   * Command that runs the feeder.
   * @param speed - Defaults to value in FeederInputs.java
   * @return command
   */
  public Command feedCommand(double speed) {
    return this.run(
      () -> feed(speed))
      .andThen(stopFeederCommand());
  }
  /**
   * Command that runs the feeder.
   * @param speed - Defaults to value in FeederInputs.java
   * @return command
   */
  public Command feedCommand() {
    return this.run(
      () -> feed())
      .andThen(stopFeederCommand());
  }

  public Command stopFeederCommand() {
    return this.runOnce(
      () -> stop()
    );
  }

}
