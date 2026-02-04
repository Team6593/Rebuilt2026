// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    candle.setControl(new SolidColor(0, 3).withColor(LEDConstants.kGreen));
    candle.setControl(new SolidColor(4, 7).withColor(LEDConstants.kWhite));

    m_anim0Chooser.setDefaultOption("Color Flow", AnimationType.ColorFlow);
    m_anim0Chooser.addOption("Rainbow", AnimationType.Rainbow);
    m_anim0Chooser.addOption("Twinkle", AnimationType.Twinkle);
    m_anim0Chooser.addOption("Twinkle Off", AnimationType.TwinkleOff);
    m_anim0Chooser.addOption("Fire", AnimationType.Fire);

    m_anim1Chooser.setDefaultOption("Larson", AnimationType.Larson);
    m_anim1Chooser.addOption("RGB Fade", AnimationType.RgbFade);
    m_anim1Chooser.addOption("Single Fade", AnimationType.SingleFade);
    m_anim1Chooser.addOption("Strobe", AnimationType.Strobe);
    m_anim1Chooser.addOption("Fire", AnimationType.Fire);

    SmartDashboard.putData("Animation 0", m_anim0Chooser);
    SmartDashboard.putData("Animation 1", m_anim1Chooser);

  }

  // Methods

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    automaticSet();
  }

  public void automaticSet() {
    final var anim0selection = m_anim0Chooser.getSelected();
    if (m_anim0State != anim0selection) {
      m_anim0State = anim0selection;

        switch (m_anim0State) {
          default:
          case ColorFlow:
              candle.setControl(
                new ColorFlowAnimation(LEDConstants.kSlot0StartIndex, LEDConstants.kSlot0EndIndex)
                  .withSlot(0)
                    .withColor(LEDConstants.kViolet)
              );
            break;
          case Rainbow:
              candle.setControl(
                new RainbowAnimation(LEDConstants.kSlot0StartIndex, LEDConstants.kSlot0EndIndex)
                  .withSlot(0)
              );
            break;
          case Twinkle:
              candle.setControl(
                new TwinkleAnimation(LEDConstants.kSlot0StartIndex, LEDConstants.kSlot0EndIndex)
                  .withSlot(0)
                    .withColor(LEDConstants.kViolet)
              );
            break;
          case TwinkleOff:
              candle.setControl(
                new TwinkleOffAnimation(LEDConstants.kSlot0StartIndex, LEDConstants.kSlot0EndIndex)
                  .withSlot(0)
                    .withColor(LEDConstants.kViolet)
              );
            break;
          case Fire:
              candle.setControl(
                new ColorFlowAnimation(LEDConstants.kSlot0StartIndex, LEDConstants.kSlot0EndIndex)
                  .withSlot(0)
              );
            break;
        }
    }

    final var anim1selection = m_anim1Chooser.getSelected();
    if (m_anim1State != anim1selection) {
      m_anim1State = anim1selection;

      switch (m_anim0State) {
        default:
        case Larson:
            candle.setControl(
              new LarsonAnimation(LEDConstants.kSlot1StartIndex, LEDConstants.kSlot1EndIndex)
                .withSlot(1)
                  .withColor(LEDConstants.kRed)
            );
          break;
        case RgbFade:
            candle.setControl(
              new RgbFadeAnimation(LEDConstants.kSlot1StartIndex, LEDConstants.kSlot1EndIndex)
                .withSlot(1)
            );
          break;
        case SingleFade:
            candle.setControl(
              new SingleFadeAnimation(LEDConstants.kSlot1StartIndex, LEDConstants.kSlot1EndIndex)
                .withSlot(1)
                  .withColor(LEDConstants.kRed)
            );
          break;
        case Strobe:
            candle.setControl(
              new StrobeAnimation(LEDConstants.kSlot1StartIndex, LEDConstants.kSlot1EndIndex)
                .withSlot(1)
                  .withColor(LEDConstants.kRed)
            );
          break;
        case Fire:
            candle.setControl(
              new FireAnimation(LEDConstants.kSlot1StartIndex, LEDConstants.kSlot1EndIndex)
                .withSlot(1)
                  .withDirection(AnimationDirectionValue.Backward)
                  .withCooling(.4)
                  .withSparking(.5)
            );
          break;
        
      }
    }
  }

  /**
   * Method that sets the control for the LEDs on slot 0. Note that most likely we'll have to disable the automatic chooser code.
   * @param animationType
   */
  public void setSlot0Control(AnimationType animationType) {
    switch (animationType) {
      default:
        case ColorFlow:
          candle.setControl(
            new ColorFlowAnimation(LEDConstants.kSlot0StartIndex, LEDConstants.kSlot0EndIndex)
              .withSlot(0)
                .withColor(LEDConstants.kViolet)
          );
          break;
        case Rainbow:
          candle.setControl(
            new RainbowAnimation(LEDConstants.kSlot0StartIndex, LEDConstants.kSlot0EndIndex)
              .withSlot(0)
          );
          break;
        case Twinkle:
          candle.setControl(
            new TwinkleAnimation(LEDConstants.kSlot0StartIndex, LEDConstants.kSlot0EndIndex)
              .withSlot(0)
                .withColor(LEDConstants.kViolet)
          );
          break;
        case TwinkleOff:
          candle.setControl(
            new TwinkleOffAnimation(LEDConstants.kSlot0StartIndex, LEDConstants.kSlot0EndIndex)
              .withSlot(0)
                .withColor(LEDConstants.kViolet)
          );
        break;
      case Fire:
          candle.setControl(
            new ColorFlowAnimation(LEDConstants.kSlot0StartIndex, LEDConstants.kSlot0EndIndex)
              .withSlot(0)
          );
        break;
    }
  }
  
  /**
   * Method that sets the control for the LEDs on slot 1. Note that most likely we'll have to disable the automatic chooser code.
   * @param animationType
   */
  public void setSlot1Control(AnimationType animationType) {
    switch (animationType) {
        default:
        case Larson:
            candle.setControl(
              new LarsonAnimation(LEDConstants.kSlot1StartIndex, LEDConstants.kSlot1EndIndex)
                .withSlot(1)
                  .withColor(LEDConstants.kRed)
            );
          break;
        case RgbFade:
            candle.setControl(
              new RgbFadeAnimation(LEDConstants.kSlot1StartIndex, LEDConstants.kSlot1EndIndex)
                .withSlot(1)
            );
          break;
        case SingleFade:
            candle.setControl(
              new SingleFadeAnimation(LEDConstants.kSlot1StartIndex, LEDConstants.kSlot1EndIndex)
                .withSlot(1)
                  .withColor(LEDConstants.kRed)
            );
          break;
        case Strobe:
            candle.setControl(
              new StrobeAnimation(LEDConstants.kSlot1StartIndex, LEDConstants.kSlot1EndIndex)
                .withSlot(1)
                  .withColor(LEDConstants.kRed)
            );
          break;
        case Fire:
            candle.setControl(
              new FireAnimation(LEDConstants.kSlot1StartIndex, LEDConstants.kSlot1EndIndex)
                .withSlot(1)
                  .withDirection(AnimationDirectionValue.Backward)
                  .withCooling(.4)
                  .withSparking(.5)
            );
          break;
        
      }
  }

  // Commands

  /**
   * Command that sets the control for the LEDs on slot 0. Note that most likely we'll have to disable the automatic chooser code.
   * @param animationType
   * @return - Command to set the control for the LEDs on slot 0.
   */
  public Command setSlot0ControlCommand(AnimationType animationType) {
    return this.runOnce(() -> setSlot0Control(animationType));
  }

  /**
   * Command that sets the control for the LEDs on slot 1. Note that most likely we'll have to disable the automatic chooser code.
   * @param animationType
   * @return - Command to set the control for the LEDs on slot 1.
   */
  public Command setSlot1ControlCommand(AnimationType animationType) {
    return this.runOnce(() -> setSlot1Control(animationType));
  }

}
