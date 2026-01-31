// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.simulation.commands.intake.PivotToHomeSimulationCommand;
import frc.simulation.commands.intake.PivotToSetpointSimulationCommand;
import frc.simulation.commands.shooter.ShooterSOTFSimulationCommand;
import frc.simulation.commands.shooter.ShooterSimulationRPMCommand;
import frc.simulation.commands.shooter.ShooterSimulationShoot;
import frc.simulation.subsystems.intake.IntakeSubsystemSimulation;
import frc.simulation.subsystems.shooter.ShooterInputsSimulation;
import frc.simulation.subsystems.shooter.ShooterSubsystemSimulation;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private Field2d field = new Field2d();

    private final RobotContainer m_robotContainer;

    private ShooterSubsystemSimulation shooterSimulation = new ShooterSubsystemSimulation();
    private IntakeSubsystemSimulation intakeSimulation = new IntakeSubsystemSimulation();

    private CommandXboxController simJoystick = new CommandXboxController(3);

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    private final boolean kUseLimelight = false;

    public Robot() {
        m_robotContainer = new RobotContainer();
        SmartDashboard.putData(field);
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        field.setRobotPose(m_robotContainer.drivetrain.getState().Pose);
        SmartDashboard.putNumber("Battery", RobotController.getBatteryVoltage());

        /*
         * This example of adding Limelight is very simple and may not be sufficient for on-field use.
         * Users typically need to provide a standard deviation that scales with the distance to target
         * and changes with number of tags available.
         *
         * This example is sufficient to show that vision integration is possible, though exact implementation
         * of how to use vision should be tuned per-robot and to the team's specification.
         */
        if (kUseLimelight) {
            var driveState = m_robotContainer.drivetrain.getState();
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

            LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
            var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
                m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
            }
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {

        simJoystick.x().whileTrue(new ShooterSimulationRPMCommand(shooterSimulation));
        simJoystick.a().whileTrue(new ShooterSOTFSimulationCommand(shooterSimulation));
        simJoystick.b().onTrue(new PivotToSetpointSimulationCommand(intakeSimulation));
        simJoystick.y().onTrue(new PivotToHomeSimulationCommand(intakeSimulation));

    }
}
