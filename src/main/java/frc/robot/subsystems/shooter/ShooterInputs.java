package frc.robot.subsystems.shooter;

public class ShooterInputs {

    // Speeds
    public static double kShooterSpeed = 1;
    public static double kIndexerSpeed = 1;

    // control (recalc: https://www.reca.lc/flywheel?currentLimit=%7B%22s%22%3A80%2C%22u%22%3A%22A%22%7D&efficiency=100&flywheelMomentOfInertia=%7B%22s%22%3A3%2C%22u%22%3A%22in2%2Albs%22%7D&flywheelRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&flywheelRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Reduction%22%7D&flywheelWeight=%7B%22s%22%3A1.5%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22Kraken%20X60%22%7D&motorRatio=%7B%22magnitude%22%3A1%2C%22ratioType%22%3A%22Step-up%22%7D&projectileRadius=%7B%22s%22%3A2%2C%22u%22%3A%22in%22%7D&projectileWeight=%7B%22s%22%3A0.5%2C%22u%22%3A%22lbs%22%7D&shooterMomentOfInertia=%7B%22s%22%3A4.5%2C%22u%22%3A%22in2%2Albs%22%7D&shooterRadius=%7B%22s%22%3A3%2C%22u%22%3A%22in%22%7D&shooterTargetSpeed=%7B%22s%22%3A3000%2C%22u%22%3A%22rpm%22%7D&shooterWeight=%7B%22s%22%3A1%2C%22u%22%3A%22lbs%22%7D&useCustomFlywheelMoi=0&useCustomShooterMoi=0)
    public static double kP = 0.57; // V*s/m
    public static double kV = .12; // V*s/m
    public static double kA = 0.08; // V*s^2/m
    public static final double kShooterGoalRPS = 2000 / 60; // numerator in rpm, / 60 to get rps
    public static double kS = 0.5; // V

    // pref keys
    public static final String kPKey = "kShooterP";
    public static final String kAKey = "kShooterV";
    public static final String kVKey = "kShooterA";
    public static final String kShooterFeedForwardKey = "kShooterFeedForward";
    public static final String kShooterSpeedKey = "kShooterSpeedKey";
    public static final String kIndexerSpeedKey = "kIndexerSpeedKey";

}
