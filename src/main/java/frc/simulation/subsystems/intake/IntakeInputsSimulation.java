package frc.simulation.subsystems.intake;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeInputsSimulation {
    public static LoggedNetworkNumber pivotKP = new LoggedNetworkNumber("/Tuning/pivotKP", 10);
    public static LoggedNetworkNumber pivotKV = new LoggedNetworkNumber("/Tuning/pivotKV", .1);
    public static LoggedNetworkNumber pivotCruiseVelocity = new LoggedNetworkNumber("/Tuning/pivotCruiseVelocity", 120);
    public static LoggedNetworkNumber pivotMaxAcceleration = new LoggedNetworkNumber("/Tuning/pivotMaxAcceleration", 1);
    public static LoggedNetworkNumber pivotAllowedProfileError = new LoggedNetworkNumber("/Tuning/pivotAllowedProfileError", .1);
}
