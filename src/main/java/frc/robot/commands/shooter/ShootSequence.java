// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.rollers.RollersSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootSequence extends Command {

  private ShooterSubsystem shooterSubsystem;
  private RollersSubsystem rollersSubsystem;
  private double shooterRPM;

  /** Creates a new ShootSequence. */
  public ShootSequence(ShooterSubsystem shooterSubsystem, RollersSubsystem rollersSubsystem, double shooterRPM) {
    this.shooterSubsystem = shooterSubsystem;
    this.rollersSubsystem = rollersSubsystem;
    this.shooterRPM = shooterRPM;

    addRequirements(shooterSubsystem, rollersSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setMasterRPM(shooterRPM, shooterRPM);
    if (shooterSubsystem.getShooterRPM() > shooterRPM) {
        rollersSubsystem.set(.25);
        shooterSubsystem.setIndexerRPM(-3700);
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    rollersSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
