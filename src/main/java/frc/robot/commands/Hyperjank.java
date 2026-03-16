// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight.LimelightConstants;

/*
 * HyperJank TM 2026 (C)
 * This awful code was written by Mansour
 */
public class Hyperjank extends Command {
  CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  /** Creates a new RotateSwerve. */
  public Hyperjank(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain =drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // just use a with-timeout cuz this is bad and idc
    System.out.println("rip kelvin ~ hyperjank");
    drivetrain.setControl(
        robotCentric.withRotationalRate(.3 * LimelightConstants.MaxAngularRate)
    );
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
