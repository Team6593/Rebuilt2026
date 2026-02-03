package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.util.Color;

public class LEDConstants {

    // candle ID
    public static final int candleID = 51;

    // colors
    public static final RGBWColor kGreen = new RGBWColor(0, 217, 0, 0);
    public static final RGBWColor kWhite = new RGBWColor(Color.kWhite).scaleBrightness(0.5);
    public static final RGBWColor kViolet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);
    public static final RGBWColor kRed = RGBWColor.fromHex("#D9000000").orElseThrow();

    // Index
    public static final int kSlot0StartIndex = 8;
    public static final int kSlot0EndIndex = 37;
    public static final int kSlot1StartIndex = 38;
    public static final int kSlot1EndIndex = 67;

    public enum AnimationType {
        None,
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff
    }

}
