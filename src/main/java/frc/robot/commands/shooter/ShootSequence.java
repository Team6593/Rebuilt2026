// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootSequence extends Command {

  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private FeederSubsystem feederSubsystem;

  /** Creates a new ShootSequence. */
  public ShootSequence(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem) {
    this.feederSubsystem = feederSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem, feederSubsystem, shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setShooterRPM(-2250);
    shooterSubsystem.setIndexerRPM(3000);

    if (shooterSubsystem.getShooterRPM() < -2200 & shooterSubsystem.getIndexerRPM() > 3000) {
        feederSubsystem.feed(.5);
        intakeSubsystem.runIntake(.45);
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    intakeSubsystem.stop();
    feederSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
