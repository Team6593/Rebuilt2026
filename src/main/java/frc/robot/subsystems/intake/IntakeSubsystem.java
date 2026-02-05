// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SubsystemInterface;

public class IntakeSubsystem extends SubsystemBase implements IntakeConstants, SubsystemInterface {

  // Devices
  private SparkMax intakeMotor = new SparkMax(intakeMotorID, MotorType.kBrushless);
  private SparkMax pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);
  private SparkMax pivot2Motor = new SparkMax(pivotMotor2ID, MotorType.kBrushless);

  private SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();
  private SparkMaxConfig pivotConfig = new SparkMaxConfig();
  private SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();

  private ProfiledPIDController pidController = new ProfiledPIDController(IntakeInputs.kPivotP, 0, 0, new TrapezoidProfile.Constraints(10, 10));
  private ArmFeedforward armFeedforward = new ArmFeedforward(.5, 12, 12.5);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    pivotConfig
      .closedLoop
        .p(IntakeInputs.kPivotP)
        .i(0)
          .feedForward
            .kV(.126);
    pivotConfig.encoder.positionConversionFactor(360.0);
    pivotConfig.absoluteEncoder.positionConversionFactor(360.0);
    pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    pivot2Motor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    Preferences.initDouble(IntakeInputs.kIntakeSpeedKey, IntakeInputs.kIntakeSpeed);
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
    SmartDashboard.putNumber("Pivot Voltage", pivotMotor.getBusVoltage());
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

  public void pidToSetpoint(double setpoint, double p) {
    pidController.setP(p);
    pidController.setGoal(setpoint);
    var pidOutput = 
      pidController.calculate(
        pivotEncoder.getPosition(), Units.degreesToRadians(setpoint));
    var feedForwardOutput =
      armFeedforward.calculate(setpoint, pidController.getSetpoint().velocity);
    pivotMotor.setVoltage(pidOutput);
  }

  public boolean pidAtSetpoint() {
    return pidController.atSetpoint();
  }

  public boolean atGoal() {
    pidController.setTolerance(5);
    return pidController.atSetpoint();
  }

  /**
   * Pivots the intake to a setpoint.
   * @param setpoint - Defaults to value in IntakeInputs.java
   * @param p
   */
  public void pivotToSetpoint(double setpoint, double p) {
    // pivotController.setSetpoint(setpoint, ControlType.kPosition);
    pivotConfig.closedLoop.p(p);
    pivot2Motor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    pivotController.setSetpoint(setpoint, ControlType.kPosition);
  }

  public boolean ihatemylife(double setpoint) {
    double tolerance = 5;
    double error = Math.abs(pivotEncoder.getPosition() - setpoint);
    if (error < tolerance) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Returns if the pivot is at the setpoint.
   * @param setpoint
   * @return boolean True if at setpoint, False if otherwise.
   */
  public boolean atSetpoint(double setpoint) {
    // return pivotController.isAtSetpoint();
    double tolerance = 3;
    double error = Math.abs(pivotEncoder.getPosition() - setpoint);
    if (error < tolerance) {
      return true;
    } else {
      return false;
    }
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
   * Command that stops the intake.
   * @return
   */
  public Command stopIntakeCommand() {
    return this.runOnce(
      () -> stop());
  }

}
