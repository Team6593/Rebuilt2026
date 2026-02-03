// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SubsystemInterface;

public class IntakeSubsystem extends SubsystemBase implements IntakeConstants, SubsystemInterface {

  // Devices
  private SparkMax intakeMotor = new SparkMax(intakeMotorID, MotorType.kBrushless);
  private SparkMax pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);

  private SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();
  private SparkFlexConfig pivotConfig = new SparkFlexConfig();
  private RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    pivotConfig
      .closedLoop
        .p(IntakeInputs.kPivotP);
    pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    Preferences.initDouble(IntakeInputs.kIntakeSpeedKey, IntakeInputs.kIntakeSpeed);
    Preferences.initDouble(IntakeInputs.kPivotPKey, IntakeInputs.kPivotP);
    Preferences.initDouble(IntakeInputs.kPivotPositionKey, IntakeInputs.kPivotPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    smartdashboardLogging();
    loadPreferences();
  }

  // Methods

  /**
   * Logs relevant information to SD.
   */
  @Override
  public void smartdashboardLogging() {
    SmartDashboard.putNumber("Intake Duty Cycle", intakeMotor.getAppliedOutput());
    SmartDashboard.putNumber("Intake Applied Output A", intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake Temp (F)", ((intakeMotor.getMotorTemperature()) * 1.8) + 32);
    SmartDashboard.putNumber("Pivot Duty Cycle", pivotMotor.getAppliedOutput());
    SmartDashboard.putNumber("Pivot Applied Output A", pivotMotor.getOutputCurrent());
    SmartDashboard.putNumber("Pivot Temp (F)", ((pivotMotor.getMotorTemperature()) * 1.8) + 32);
    SmartDashboard.putNumber("Pivot Position", pivotEncoder.getPosition());
  }

  /**
   * Loads the preference keys for tuning.
   */
  @Override
  public void loadPreferences() {
    if (IntakeInputs.kIntakeSpeed != Preferences.getDouble(IntakeInputs.kIntakeSpeedKey, IntakeInputs.kIntakeSpeed)) {
      System.out.println("Old kIntakeSpeed: " + IntakeInputs.kIntakeSpeed);
      IntakeInputs.kIntakeSpeed = Preferences.getDouble(IntakeInputs.kIntakeSpeedKey, IntakeInputs.kIntakeSpeed);
      System.out.println("New kIntakeSpeed: " + IntakeInputs.kIntakeSpeed);
    }
    if (IntakeInputs.kPivotP != Preferences.getDouble(IntakeInputs.kPivotPKey, IntakeInputs.kPivotP)) {
      System.out.println("Old kPivotP: " + IntakeInputs.kPivotP);
      IntakeInputs.kPivotP = Preferences.getDouble(IntakeInputs.kPivotPKey, IntakeInputs.kPivotP);
      pivotConfig.closedLoop.p(IntakeInputs.kPivotP);
      System.out.println("New kPivotP: " + IntakeInputs.kPivotP);
    }
    if (IntakeInputs.kPivotPosition != Preferences.getDouble(IntakeInputs.kPivotPositionKey, IntakeInputs.kPivotPosition)) {
      System.out.println("Old kIntakeSpeed: " + IntakeInputs.kPivotPosition);
      IntakeInputs.kPivotPosition = Preferences.getDouble(IntakeInputs.kPivotPositionKey, IntakeInputs.kPivotPosition);
      System.out.println("New kPivotPosition: " + IntakeInputs.kPivotPosition);
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
  public void intake() {
    intakeMotor.set(IntakeInputs.kIntakeSpeed);
  }

  /**
   * Pivots the intake to a setpoint.
   * @param setpoint - Defaults to value in IntakeInputs.java
   */
  public void pivotToSetpoint(double setpoint) {
    // pivotController.setSetpoint(setpoint, ControlType.kPosition);
    pivotController.setSetpoint(setpoint, ControlType.kPosition);
  }

  /**
   * Pivots the intake to a setpoint.
   * @param setpoint - Defaults to value in IntakeInputs.java
   */
  public void pivotToSetpoint() {
    pivotController.setSetpoint(IntakeInputs.kPivotPosition, ControlType.kPosition);
  }

  /**
   * Returns if the pivot is at the setpoint.
   * @return boolean True if at setpoint, False if otherwise.
   */
  public boolean atSetpoint() {
    return pivotController.isAtSetpoint();
  }

  public void pivot(double speed) {
    pivotMotor.set(speed);
  }

  /**
   * Stops the intake.
   */
  @Override
  public void stop() {
    intakeMotor.stopMotor();
    pivotMotor.stopMotor();
  }

  // Commands

  /**
   * Command that runs the intake.
   * @param speed - Defaults to value in IntakeInputs.java
   * @return command
   */
  public Command runIntakeCommand(double speed) {
    return this.run(
      () -> runIntake(speed))
      .andThen(stopIntakeCommand());
  }

  /**
   * Command that runs the intake.
   * @param speed - Defaults to value in IntakeInputs.java
   * @return command
   */
  public Command runIntakeCommand() {
    return this.run(
      () -> intake())
      .andThen(stopIntakeCommand());
  }

  /**
   * Command that reverses the intake.
   * @param speed - Defaults to value in IntakeInputs.java
   * @return command
   */
  public Command reverseIntakeCommand(double speed) {
    return this.run(
      () -> runIntake(-speed))
      .andThen(stopIntakeCommand());
  }

  /**
   * Command that reverses the intake.
   * @param speed - Defaults to value in IntakeInputs.java
   * @return command
   */
  public Command reverseIntakeCommand() {
    return this.run(
      () -> intake())
      .andThen(stopIntakeCommand());
  }

  /**
   * Command that pivots the pivot to the setpoint.
   * @param setpoint - Defaults to value in IntakeInputs.java
   * @return
   */
  public Command pivotToSetpointCommand(double setpoint) {
    return this.run(
      () -> pivotToSetpoint(setpoint))
      .andThen(stopIntakeCommand());
  }

  /**
   * Command that pivots the pivot to the setpoint.
   * @param setpoint - Defaults to value in IntakeInputs.java
   * @return
   */
  public Command pivotToSetpointCommand() {
    return this.run(
      () -> pivotToSetpoint())
      .andThen(stopIntakeCommand());
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
