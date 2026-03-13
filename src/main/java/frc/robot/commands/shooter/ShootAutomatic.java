// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.rollers.RollersSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.ShotCalculator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootAutomatic extends Command {

  private ShooterSubsystem shooterSubsystem;
  private RollersSubsystem rollersSubsystem;

  /** Creates a new ShootSequence. */
  public ShootAutomatic(ShooterSubsystem shooterSubsystem, RollersSubsystem rollersSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.rollersSubsystem = rollersSubsystem;

    addRequirements(shooterSubsystem, rollersSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("SHOOTING INIT");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setMasterRPM(ShotCalculator.lerpGet(Limelight.staticDistanceGet()).rpm, ShotCalculator.lerpGet(Limelight.staticDistanceGet()).rpm);
    if (shooterSubsystem.getShooterRPM() > ShotCalculator.lerpGet(Limelight.staticDistanceGet()).rpm) {
        rollersSubsystem.set(.25);
        shooterSubsystem.setIndexerRPM(-3700);
    } 
    System.out.println("SHOOTING!!!!!!!!! ~ nafi");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
    rollersSubsystem.stop();
    System.out.println("SHOOTING END");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
