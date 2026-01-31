package frc.simulation.subsystems.shooter;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterInputsSimulation {
    public static LoggedNetworkNumber shooterRPM = new LoggedNetworkNumber("/Tuning/shooterRPM", 2200);
    public static LoggedNetworkNumber shooterFeedForward = new LoggedNetworkNumber("/Tuning/shooterFeedForward", .5);
    public static LoggedNetworkNumber shooterKP = new LoggedNetworkNumber("/Tuning/shooterKP", .49);
    public static LoggedNetworkNumber shooterKV = new LoggedNetworkNumber("/Tuning/shooterKV", .12);
    public static LoggedNetworkNumber shooterKS = new LoggedNetworkNumber("/Tuning/shooterKA", .06);
    public static LoggedNetworkNumber shooterKA = new LoggedNetworkNumber("/Tuning/shooterKS", .5);
    public static LoggedNetworkNumber shooterKdt = new LoggedNetworkNumber("/Tuning/shooterKdt", 0.02);
    public static double goalRPM = 1000;
    public static String goalRPMKey = "GoalRPMKey";
}
