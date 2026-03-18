// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.limelight.LimelightConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Alignment extends Command {

  private CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(LimelightConstants.MaxSpeed * 0.1).withRotationalDeadband(LimelightConstants.MaxAngularRate * 0.05) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private double rotKP;
  private double velocityX;
  private double velocityY;

  /**
   * Alignment command for shooting.
   * @param drivetrain - drivetrain
   * @param rotKP - kP for rotation | rotation is TX * kHubAngle * rotkP
   * @param velocityX - X velocity of drivetrain
   * @param velocityY - Y velocity of drivetrain
   */
  public Alignment(CommandSwerveDrivetrain drivetrain, double rotKP, double velocityX, double velocityY) {
    this.drivetrain = drivetrain;
    this.rotKP = rotKP;
    this.velocityX = velocityX;
    this.velocityY = velocityY;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("ALIGNING");
    SmartDashboard.putBoolean("Aligning", true);
    if (Limelight.staticValidTargets("limelight")) {
      drivetrain.applyRequest(
        () -> drive
          .withRotationalRate(LimelightHelpers.getTX("limelight") * -LimelightConstants.kHubAngle * rotKP)
          .withVelocityX(velocityX)
          .withVelocityY(velocityY)
      );
    } else if (Limelight.staticValidTargets("limelight-two")) {
      drivetrain.applyRequest(
        () -> drive
          .withRotationalRate(LimelightHelpers.getTX("limelight") * -LimelightConstants.kHubAngle * rotKP)
          .withVelocityX(velocityX)
          .withVelocityY(velocityY)
      );
    } else {
      drivetrain.applyRequest(
        () -> drive
          .withRotationalRate(LimelightHelpers.getTX("limelight") * -LimelightConstants.kHubAngle * rotKP)
          .withVelocityX(velocityX)
          .withVelocityY(velocityY)
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(
      () -> drive
        .withRotationalRate(0)
        .withVelocityX(0)
        .withVelocityY(0) 
    );
    SmartDashboard.putBoolean("Aligning", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
