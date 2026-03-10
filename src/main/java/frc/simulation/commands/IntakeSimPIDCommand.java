// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.simulation.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.simulation.intake.IntakeSimulation;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeSimPIDCommand extends Command {

  private IntakeSimulation intakeSimulation;
  private double setpoint;
  private double kP;

  /** Creates a new IntakePIDCommand. */
  public IntakeSimPIDCommand(IntakeSimulation intakeSimulation, double setpoint, double kP) {
    this.intakeSimulation = intakeSimulation;
    this.setpoint = setpoint;
    this.kP = kP;

    addRequirements(intakeSimulation);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSimulation.pidToSetpoint(setpoint, kP);
    System.out.println(intakeSimulation.atSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSimulation.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSimulation.atSetpoint();
  }
}
