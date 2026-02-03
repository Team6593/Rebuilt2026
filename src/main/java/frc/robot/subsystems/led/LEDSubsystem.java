// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.led.LEDConstants.AnimationType;

public class LEDSubsystem extends SubsystemBase {

  private final CANdle candle = new CANdle(LEDConstants.candleID, "rio");

  private AnimationType m_anim0State = AnimationType.None;
  private AnimationType m_anim1State = AnimationType.None;

  private final SendableChooser<AnimationType> m_anim0Chooser = new SendableChooser<AnimationType>();
  private final SendableChooser<AnimationType> m_anim1Chooser = new SendableChooser<AnimationType>();

  /** Creates a new LEDsSubsystem. */
  public LEDSubsystem() {

    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.GRB;
    config.LED.BrightnessScalar = 1;
    config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
    candle.getConfigurator().apply(config);

    for (int i = 0; i < 8; ++i) {
      candle.setControl(new EmptyAnimation(i));
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
