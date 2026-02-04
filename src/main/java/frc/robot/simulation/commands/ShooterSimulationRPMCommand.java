// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.simulation.shooter.ShooterSimulation;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterSimulationRPMCommand extends Command {

  private ShooterSimulation shooterSimulation;

  /** Creates a new ShooterRPMCommand. */
  public ShooterSimulationRPMCommand(ShooterSimulation shooterSimulation) {
    this.shooterSimulation = shooterSimulation;

    addRequirements(shooterSimulation);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSimulation.setRPM(-1950);
    if (shooterSimulation.getRPM() < -1900) {
      // insert feeder logic here and such
      shooterSimulation.setRPM(shooterSimulation.getRPM() + 50);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSimulation.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
