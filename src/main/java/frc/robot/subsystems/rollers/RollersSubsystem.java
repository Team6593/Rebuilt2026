// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollers;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SubsystemInterface;

public class RollersSubsystem extends SubsystemBase implements SubsystemInterface{

  private SparkFlex rollerMotor = new SparkFlex(RollerConstants.rollerMotorID, MotorType.kBrushless);

  /** Creates a new RollersSubsystem. */
  public RollersSubsystem() {
    Preferences.initDouble(RollerInputs.kRollerSpeedKey, RollerInputs.kRollerSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    loadPreferences();
    sdLogging();
  }

  // Methods/functions
  // Interface methods

  @Override
  public void stop() {
    rollerMotor.stopMotor();
  }

  @Override
  public void loadPreferences() {
    if (RollerInputs.kRollerSpeed != Preferences.getDouble(RollerInputs.kRollerSpeedKey, RollerInputs.kRollerSpeed)) {
      System.out.println("Old kRollerSpeed: " + RollerInputs.kRollerSpeed);
      RollerInputs.kRollerSpeed = Preferences.getDouble(RollerInputs.kRollerSpeedKey, RollerInputs.kRollerSpeed);
      System.out.println("New kRollerSpeed: " + RollerInputs.kRollerSpeed);
    }
  }

  @Override
  public void sdLogging() {
    SmartDashboard.putNumber("Roller V", rollerMotor.getBusVoltage());
    SmartDashboard.putNumber("Roller DutyCycle", rollerMotor.get());
    SmartDashboard.putNumber("Roller Temperature (F)", ((rollerMotor.getMotorTemperature()) * 1.8) + 32);
  }

  // Subsytem methods

  /**
   * Runs the roller at specified speed.
   * @param speed - Defaults to value in {@link RollerInputs}
   */
  public void set(double speed) {
    rollerMotor.set(speed);
  }

  /**
   * Runs the roller at specified speed.
   * @param speed - Defaults to value in {@link RollerInputs}
   */
  public void set() {
    rollerMotor.set(RollerInputs.kRollerSpeed);
  }  

  // Commands

  /**
   * Command that sets the speed of the rollers.
   * @param speed - Defaults to value in {@link RollerInputs}
   * @return {@link Command}
   */
  public Command setSpeedCommand(double speed) {
    return this.runEnd(
      () -> set(speed), 
      () -> stop());
  }

  /**
   * Command that sets the speed of the rollers.
   * @param speed - Defaults to value in {@link RollerInputs}
   * @return {@link Command}
   */
  public Command setSpeedCommand() {
    return this.runEnd(
      () -> set(), 
      () -> stop());
  }

  /**
   * Command that stops the rollers.
   * @return {@link Command}
   */
  public Command stopRollerCommand() {
    return this.runOnce(
      () -> stop());
  }

}
