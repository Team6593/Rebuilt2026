// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.rollers.RollersSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopAll extends InstantCommand {

  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private RollersSubsystem rollers;

  /**
   * Command that stops all subsystems.
   * @param intake
   * @param shooter
   */
  public StopAll(IntakeSubsystem intake, ShooterSubsystem shooter, RollersSubsystem rollers) {
    this.shooter = shooter;
    this.intake = intake;
    this.rollers = rollers;

    addRequirements(shooter, intake, rollers);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.stop();
    intake.stop();
    rollers.stop();
  }
}
