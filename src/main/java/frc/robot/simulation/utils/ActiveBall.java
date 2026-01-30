package frc.robot.simulation.utils;

import edu.wpi.first.math.geometry.Pose2d;

public class ActiveBall {
    public double startTime;
    public Pose2d startPose;
    public double velocity;

    public ActiveBall(double startTime, Pose2d startPose, double velocity) {
        this.startTime = startTime;
        this.startPose = startPose;
        this.velocity = velocity;
    }
}
