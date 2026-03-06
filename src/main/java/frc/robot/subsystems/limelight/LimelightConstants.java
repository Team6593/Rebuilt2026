package frc.robot.subsystems.limelight;
import static edu.wpi.first.units.Units.*;
import frc.robot.generated.TunerConstants;

public class LimelightConstants {

    public static double kAimP = .035;

    // estimate variables
    public static double limelightMountAngleDegrees = -25;
    public static double limelightLensHeightInches = 8.3125;
    public static double goalHeightInches = 41;

    // drive speeds
    public static double drivetrainMaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    // get actual value later; https://youtu.be/QXU7Co__Xdc ~13:00
    public static double kHubAngle = -2.5;
    public static double kHubTrueAngle = 1.76;
    public static double sotmRotMulti = .275;
    public static double kTrenchAngle = -.13;

}

