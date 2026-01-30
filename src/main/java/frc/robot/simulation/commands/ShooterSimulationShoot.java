// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation.commands;

import static edu.wpi.first.units.Units.Rotation;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.simulation.shooter.ShooterSimulation;
import frc.robot.simulation.utils.FieldSimulation;
import frc.robot.simulation.utils.ActiveBall;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterSimulationShoot extends Command {

  private ShooterSimulation shooterSimulation;
  private final List<ActiveBall> activeBalls = new ArrayList<>(); 
  private List<Pose3d> ballPoses3d = new ArrayList<>();
  private List<Pose2d> ballPoses2d = new ArrayList<>();
  private double flightTime = 0;
  private boolean ballInFlight = false;
  private final double kLaunchAngleRad = Math.toRadians(45);

  /** Creates a new ShooterSimulationShoot. */
  public ShooterSimulationShoot(ShooterSimulation shooterSimulation) {
    this.shooterSimulation = shooterSimulation;

    addRequirements(shooterSimulation);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ballInFlight = true;
    flightTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    shooterSimulation.shoot(1);
    var iterator = activeBalls.iterator();
    while (iterator.hasNext()) {
      ActiveBall ball = iterator.next();
      double groundDistance = (ball.velocity * Math.cos(kLaunchAngleRad)) * flightTime;
      double heightZ = (ball.velocity * Math.sin(kLaunchAngleRad)) * flightTime - 
        (0.5 * 9.8 * Math.pow(flightTime, 2));

      if (heightZ < 0 && flightTime > .1) {
        iterator.remove();
        continue;
      }
      Rotation2d heading = ball.startPose.getRotation();
      double x = ball.startPose.getX() + (groundDistance * heading.getCos());
      double y = ball.startPose.getY() + (groundDistance * heading.getSin());

      ballPoses3d.add(new Pose3d(x,y, heightZ, new Rotation3d()));
      ballPoses2d.add(new Pose2d(x, y, heading));
    }
    FieldSimulation.fieldSim.getObject("Fuel").setPoses(ballPoses2d.toArray(new Pose2d[0]));
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
