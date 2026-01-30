// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.simulation.shooter.ShooterSimulation;
import frc.robot.simulation.utils.FieldSimulation;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterSimulationRPMCommand extends Command {

  private ShooterSimulation shooterSimulation;
  private Pose2d[] ballPoses = new Pose2d[1];
  private double flightTime = 0;
  private boolean ballInFlight = false;
  private final double kLaunchAngleRad = Math.toRadians(45);

  /** Creates a new ShooterRPMCommand. */
  public ShooterSimulationRPMCommand(ShooterSimulation shooterSimulation) {
    this.shooterSimulation = shooterSimulation;

    addRequirements(shooterSimulation);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSimulation.setRPM(2000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterSimulation.getRotorVelocity() > 1900) {
      if (ballInFlight) {
        flightTime += 0.020;
        double velocity = shooterSimulation.getRotorVelocity();
        double x = (velocity * Math.cos(kLaunchAngleRad)) * flightTime;
        double z = (velocity * Math.sin(kLaunchAngleRad)) * flightTime;
        FieldSimulation.fieldSim.getObject("Fuel").setPose(new Pose2d(x, z, new Rotation2d()));
        if (z < 0) ballInFlight = false;
      }
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
