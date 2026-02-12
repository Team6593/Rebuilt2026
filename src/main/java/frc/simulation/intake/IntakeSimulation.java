// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.simulation.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSimulation extends SubsystemBase {

  private final DCMotor gearbox = DCMotor.getNEO(2);
  private final SparkMax motor1 = new SparkMax(61, MotorType.kBrushless);
  private final SparkMax motor2 = new SparkMax(62, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor1.getEncoder();

  private final ProfiledPIDController pidController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(1, 1));
  private final SingleJointedArmSim intakeSim = 
    new SingleJointedArmSim(
      gearbox, 
      9, 
      SingleJointedArmSim.estimateMOI(1, 1), 
      1, 
      Units.degreesToRadians(233), 
      Units.degreesToRadians(338), 
      true, 
      Units.degreesToRadians(338), 
      2.0 * Math.PI / 4096,
      0);
  private final Mechanism2d mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d intakePivot = mech2d.getRoot("IntakePivot", 30, 30);
  private final MechanismLigament2d intakeTower = 
    intakePivot.append(new MechanismLigament2d("IntakeTower", 10, 10));
  private final MechanismLigament2d intake = 
    intakePivot.append(new MechanismLigament2d(
      "Intake", 
      30, 
      Units.radiansToDegrees(intakeSim.getAngleRads()),
      6,
      new Color8Bit(Color.kYellow)));

  /** Creates a new IntakeSimulation. */
  public IntakeSimulation() {
    encoder.setPosition(338);
    //encod
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
