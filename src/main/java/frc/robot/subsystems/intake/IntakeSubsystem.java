// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase implements IntakeConstants{

  private SparkMax intakeMotor = new SparkMax(intakeMotorID, MotorType.kBrushless);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    Preferences.initDouble(IntakeInputs.kIntakeSpeedKey, IntakeInputs.kIntakeSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    smartdashboardlogging();
    loadPreferences();
  }

  // Methods

  public void smartdashboardlogging() {
    SmartDashboard.putNumber("Intake Duty Cycle", intakeMotor.getAppliedOutput());
    SmartDashboard.putNumber("Intake Applied Output A", intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake Temp (F)", ((intakeMotor.getMotorTemperature()) * 1.8) + 32);
  }

  public void loadPreferences() {
    if (IntakeInputs.kIntakeSpeed != Preferences.getDouble(IntakeInputs.kIntakeSpeedKey, IntakeInputs.kIntakeSpeed)) {
      System.out.println("Old kIntakeSpeed: " + IntakeInputs.kIntakeSpeed);
      IntakeInputs.kIntakeSpeed = Preferences.getDouble(IntakeInputs.kIntakeSpeedKey, IntakeInputs.kIntakeSpeed);
      System.out.println("New kIntakeSpeed: " + IntakeInputs.kIntakeSpeed);
    }
  }

  /**
   * Runs the intake.
   * @param speed - Defaults to value in IntakeInputs.java
   */
  public void runIntake(double speed) {
    intakeMotor.set(speed);
  }

  /**
   * Runs the intake.
   * @param speed - Defaults to value in IntakeInputs.java
   */
  public void runIntake() {
    intakeMotor.set(IntakeInputs.kIntakeSpeed);
  }

  /**
   * Stops the intake.
   */
  public void stop() {
    intakeMotor.stopMotor();
  }

  // Commands

  /**
   * Command that runs the intake.
   * @param speed - Defaults to value in IntakeInputs.java
   * @return command
   */
  public Command runIntakeCommand(double speed) {
    return this.run(
      () -> runIntake(speed)).andThen(stopIntakeCommand());
  }

  /**
   * Command that runs the intake.
   * @param speed - Defaults to value in IntakeInputs.java
   * @return command
   */
  public Command runIntakeCommand() {
    return this.run(
      () -> runIntake()).andThen(stopIntakeCommand());
  }

  /**
   * Command that reverses the intake.
   * @param speed - Defaults to value in IntakeInputs.java
   * @return command
   */
  public Command reverseIntakeCommand(double speed) {
    return this.run(
      () -> runIntake(-speed)).andThen(stopIntakeCommand());
  }

  /**
   * Command that reverses the intake.
   * @param speed - Defaults to value in IntakeInputs.java
   * @return command
   */
  public Command reverseIntakeCommand() {
    return this.run(
      () -> runIntake()).andThen(stopIntakeCommand());
  }

  /**
   * Command that stops the intake.
   * @return
   */
  public Command stopIntakeCommand() {
    return this.runOnce(
      () -> stop());
  }

}
