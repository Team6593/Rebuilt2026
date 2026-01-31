// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.simulation.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.simulation.subsystems.intake.IntakeSubsystemSimulation;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotToHomeSimulationCommand extends Command {

  private IntakeSubsystemSimulation intakeSubsystemSimulation;

  /** Creates a new PivotToHomeSimulationCommand. */
  public PivotToHomeSimulationCommand(IntakeSubsystemSimulation intakeSubsystemSimulation) {
    this.intakeSubsystemSimulation = intakeSubsystemSimulation;

    addRequirements(intakeSubsystemSimulation);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystemSimulation.pivotToSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystemSimulation.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSubsystemSimulation.isAtSetpoint();
  }
}
