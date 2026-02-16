// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight.LimelightConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTrench extends Command {
  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.RobotCentric robotCentric;
  
  /** Creates a new AlignToTrench. */
  public AlignToTrench(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric robotCentric) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.robotCentric = robotCentric;
  }

   double aimToTrench()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .01;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= LimelightConstants.MaxAngularRate;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Mansour note: use TX from target pose from robot space for translation
    drivetrain.setControl(
      robotCentric
        .withRotationalRate(aimToTrench())
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    System.out.println("DONE ALIGNING TO TRENCH!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = false;
    if (Math.abs(LimelightHelpers.getTX("limelight")) < .05) {
      finished = true;
    } else {
      finished = false;
    }
    return finished;
  }
}
