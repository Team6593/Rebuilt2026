// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight.LimelightConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Recenter extends Command {

  private CommandSwerveDrivetrain drivetrain;
      private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private double angle;

  /** Creates a new Recenter. */
  public Recenter(CommandSwerveDrivetrain drivetrain, double angle) {
    this.drivetrain = drivetrain;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!positionCheck()) {
      drivetrain.setControl(
        robotCentric
          .withRotationalRate(-.2 * LimelightConstants.MaxAngularRate)
      );
    }
    System.out.println("recentering lmao");
  }

  public boolean positionCheck() {
    double tolerance = 5;
    double error = Math.abs(Math.abs(drivetrain.getPigeon2().getYaw().getValueAsDouble() % 360) - angle);
    if (error < tolerance) {
      return true;
    } else {
      return false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return positionCheck();
  }
}
