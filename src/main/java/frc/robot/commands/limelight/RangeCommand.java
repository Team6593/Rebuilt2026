// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;


import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight.Limelight;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RangeCommand extends Command {

  private Limelight limelight;
  private CommandSwerveDrivetrain drivetrain;
  private RobotContainer robotContainer;
  private final SwerveRequest.RobotCentric robotCentric;

  /** Creates a new RangeCommand. */
  public RangeCommand(Limelight limelight, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric robotCentric) {
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.robotCentric = robotCentric;

    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final var rot_limelight = robotContainer.limelightAimProportional();
    double rot = rot_limelight;

    final var forwardLimelight = robotContainer.limelightRangeProportional();
    double xSpeed = forwardLimelight;

    drivetrain.setControl(
      robotCentric
        .withVelocityX(xSpeed)
        .withVelocityY(0)
        .withRotationalRate(rot));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
