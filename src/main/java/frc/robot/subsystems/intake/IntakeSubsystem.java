// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SubsystemInterface;

public class IntakeSubsystem extends SubsystemBase implements SubsystemInterface {

  // Devices
  private SparkMax intakeMotor = new SparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);
  private SparkMax pivotMotor = new SparkMax(IntakeConstants.pivotMotorID, MotorType.kBrushless);
  private SparkMax pivot2Motor = new SparkMax(IntakeConstants.pivotMotor2ID, MotorType.kBrushless);

  private SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();
  private SparkMaxConfig pivot1Config = new SparkMaxConfig();
  private SparkMaxConfig pivot2Config = new SparkMaxConfig();
  private SparkAbsoluteEncoder pivot2Encoder = pivot2Motor.getAbsoluteEncoder();
  private SparkClosedLoopController intakeController = intakeMotor.getClosedLoopController();
  private SparkMaxConfig intakeConfig = new SparkMaxConfig();

  private ProfiledPIDController pidController = new ProfiledPIDController(0.0175, 0, 0, new TrapezoidProfile.Constraints(6.545, 0));
  // private ProfiledPIDController intakePIDController = new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(6000, 10));
  // private ArmFeedforward armFeedforward = new ArmFeedforward(.5, 12, 12.5);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    pivot1Config
      .closedLoop
        .p(.0175)
        .i(0)
          .feedForward
            .kV(.126);
    pivot2Config
      .closedLoop
        .p(.0175)
        .i(0)
          .feedForward
            .kV(.126);
    pivot1Config.closedLoop.outputRange(-.15, .15);
    pivot2Config.closedLoop.outputRange(-.15, .15);
    pivot1Config.smartCurrentLimit(40);
    pivot2Config.smartCurrentLimit(40);
    pivotMotor.configure(pivot1Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    pivot2Motor.configure(pivot2Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    
    intakeConfig
      .closedLoop
        .p(2)
        .i(0)
        .d(0);
    intakeMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    pivot2Motor.getEncoder().setPosition(-100);
    Preferences.initDouble(IntakeInputs.kIntakeSpeedKey, IntakeInputs.kIntakeSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sdLogging();
    loadPreferences();
  }

  // Methods

  /**
   * Logs relevant information to SD.
   */
  @Override
  public void sdLogging() {
    SmartDashboard.putNumber("Intake Duty Cycle", intakeMotor.getAppliedOutput());
    SmartDashboard.putNumber("Intake Applied Output A", intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake Temp (F)", ((intakeMotor.getMotorTemperature()) * 1.8) + 32);
    SmartDashboard.putNumber("Intake RPM", intakeMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Intake Ft/S", (intakeMotor.getEncoder().getVelocity() * 1.125 * (Math.PI/12)));
    SmartDashboard.putNumber("Pivot1 Duty Cycle", pivotMotor.getAppliedOutput());
    SmartDashboard.putNumber("Pivot2 Duty Cycle", pivot2Motor.getAppliedOutput());
    SmartDashboard.putNumber("Pivot Applied Output A", pivotMotor.getOutputCurrent());
    SmartDashboard.putNumber("Pivot2 Applied Output A", pivot2Motor.getOutputCurrent());
    SmartDashboard.putNumber("Pivot Temp (F)", ((pivotMotor.getMotorTemperature()) * 1.8) + 32);
    SmartDashboard.putNumber("Pivot Position", pivot2Encoder.getPosition());
    SmartDashboard.putNumber("Pivot2 Rel Pos", pivot2Motor.getEncoder().getPosition());
    SmartDashboard.putNumber("Pivot Voltage", pivotMotor.getBusVoltage());
    SmartDashboard.putNumber("Pivot2 Voltage", pivot2Motor.getBusVoltage());
    SmartDashboard.putNumber("Pivot1 RPM", pivotMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Pivot2 RPM", pivot2Motor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Pivot P: ", pidController.getP());
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
   * @param speed - Defaults to value in {@link IntakeInputs}
   */
  public void runIntake(double speed) {
    intakeMotor.set(speed);
  }

  /**
   * Runs the intake.
   * @param speed - Defaults to value in {@link IntakeInputs}
   */
  public void intake() {
    intakeMotor.set(IntakeInputs.kIntakeSpeed);
  }

  public void pidToSetpoint(double setpoint, double p) {
    pidController.setP(p);
    pidController.setGoal(setpoint);
    var pidOutput = 
      pidController.calculate(
        pivot2Encoder.getPosition(), Units.degreesToRadians(setpoint));
    // @SuppressWarnings("unused")
    // var feedForwardOutput =
    //   armFeedforward.calculate(setpoint, pidController.getSetpoint().velocity);
    pivot2Motor.setVoltage(pidOutput);
    pivotMotor.setVoltage(pidOutput);
  }

  public void revPlease1(double setpoint, double p) {
    pidController.setP(p);
    pidController.setGoal(setpoint);
    var pidOutput = 
      pidController.calculate(
        pivot2Encoder.getPosition(), Units.degreesToRadians(setpoint));
    // @SuppressWarnings("unused")
    // var feedForwardOutput =
    //   armFeedforward.calculate(setpoint, pidController.getSetpoint().velocity);
    pivotMotor.setVoltage(pidOutput);
  }

  public void profiledPIDIntake(double setpoint) {
    pidController.setGoal(setpoint);
    var pidOutput = 
      pidController.calculate(
        intakeMotor.getEncoder().getVelocity(), setpoint);
    intakeMotor.setVoltage(pidOutput);
  }

  public void revINeedThis(double setpoint) {
    pivot2Motor.getClosedLoopController().setSetpoint(setpoint, ControlType.kPosition);
  }

  public boolean myRobotsKindaPivotless() {
    return pivot2Motor.getClosedLoopController().isAtSetpoint();
  }

  public void pidIntake(double setpoint) {
    intakeController.setSetpoint(setpoint, ControlType.kVelocity);
  }

  public void zeroPivot() {
  
  }

  public boolean pidAtSetpoint() {
    return pidController.atSetpoint();
  }

  public boolean atGoal() {
    pidController.setTolerance(5);
    return pidController.atSetpoint();
  }

  public double getOutputCurrent() {
    return pivot2Motor.getOutputCurrent();
  }

  public void offsetSetpoint() {
    if (!positionCheck(40)) {
      double offset = pivot2Encoder.getPosition() / 360;
      if (offset > 1) offset += 1; 
      offset += .01;
      System.out.println(offset);
      if (offset > 1) {
        offset -= 1;
      }
      pivot2Config.absoluteEncoder.zeroOffset(offset);
      pivot2Motor.configure(pivot2Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      System.out.println(offset);
      System.out.println("Offset'd.");
    }
  }

  public void offsetHome() {
    if (!positionCheck(255)) {
      double offset = pivot2Encoder.getPosition() / 360;
      if (offset > 1) offset += 1; 
      offset += .01;
      System.out.println(offset);
      if (offset > 1) {
        offset -= 1;
      }
      pivot2Config.absoluteEncoder.zeroOffset(offset);
      pivot2Motor.configure(pivot2Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      System.out.println(offset);
      System.out.println("Offset'd.");
    }
  }

  /**
   * Pivots the intake to a setpoint.
   * @param setpoint - Defaults to value in {@link IntakeInputs}
   * @param p - kP
   */
  public void pivotToSetpoint(double setpoint, double p) {
    // pivotController.setSetpoint(setpoint, ControlType.kPosition);
    pivot1Config.closedLoop.p(p);
    pivot2Motor.configure(pivot1Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    pivotMotor.configure(pivot1Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    pivotController.setSetpoint(setpoint, ControlType.kPosition);
  }

  public boolean positionCheck(double setpoint) {
    double tolerance = 10;
    double error = Math.abs(pivot2Encoder.getPosition() - setpoint);
    if (error < tolerance) {
      return true;
    } else {
      return false;
    }
  }

  public boolean homeCheck(double setpoint) {
    double tolerance = 20;
    double error = Math.abs(pivot2Encoder.getPosition() - setpoint);
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
  // public boolean atSetpoint(double setpoint) {
  //   // return pivotController.isAtSetpoint();
  //   double tolerance = 3;
  //   double error = Math.abs(pivot2Encoder.getPosition() - setpoint);
  //   if (error < tolerance) {
  //     return true;
  //   } else {
  //     return false;
  //   }
  // }

  public void pivot(double speed) {
    pivot2Motor.set(speed);
    pivotMotor.set(-speed);
  }

  public void pivot1(double speed) {
    pivotMotor.set(speed);
  }

  public void pivot2(double speed) {
    pivot2Motor.set(speed);
  }

  /**
   * Stops the intake.
   */
  @Override
  public void stop() {
    intakeMotor.stopMotor();
    pivotMotor.stopMotor();
    pivot2Motor.stopMotor();
  }

  // Commands

  /**
   * Command that runs the intake.
   * @param speed - Defaults to value in {@link IntakeInputs.java}
   * @return command
   */
  public Command runIntakeCommand(double speed) {
    return this.run(
      () -> runIntake(speed))
      .andThen(stopIntakeCommand());
  }

  /**
   * Command that runs the intake.
   * @param speed - Defaults to value in {@link IntakeInputs.java}
   * @return command
   */
  public Command runIntakeCommand() {
    return this.run(
      () -> intake())
      .andThen(stopIntakeCommand());
  }

  /**
   * Command that reverses the intake.
   * @param speed - Defaults to value in {@link IntakeInputs.java}
   * @return command
   */
  public Command reverseIntakeCommand(double speed) {
    return this.run(
      () -> runIntake(-speed))
      .andThen(stopIntakeCommand());
  }

  /**
   * Command that reverses the intake.
   * @param speed - Defaults to value in {@link IntakeInputs.java}
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

  public Command offsetCommand() {
    System.out.println("Offset running");
    return this.runOnce(
      () -> offsetSetpoint()
    );
  }

}
