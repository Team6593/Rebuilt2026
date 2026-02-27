// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.limelight.LimelightConstants;
import frc.simulation.commands.ShooterSimDistanceSetter;
import frc.simulation.commands.ShooterSimulationRPMCommand;
import frc.simulation.shooter.ShooterSimulation;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private Field2d field = new Field2d();

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    private final boolean kUseLimelight = true;

    // Simulation
    private ShooterSimulation shooterSimulation = new ShooterSimulation();
    private CommandXboxController simJoystick = new CommandXboxController(3);

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
        SmartDashboard.putNumber("Alignment Rate", LimelightHelpers.getTX("limelight") * LimelightConstants.kHubAngle * LimelightConstants.sotmRotMulti);
        SmartDashboard.putNumber("ATag ID", LimelightHelpers.getFiducialID("limelight"));
        // SmartDashboard.putNumber("Calculated RPM", ShotCalculator.lerpGet(m_robotContainer.limelight.estimateDistance()).rpm);

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
        if (LimelightHelpers.getFiducialID("limelight") == -1) {
            m_robotContainer.joystick.setRumble(RumbleType.kBothRumble, .5);
        } else {
            m_robotContainer.joystick.setRumble(RumbleType.kBothRumble, 0);
        }

        if (!RobotBase.isReal()) {
            
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

        // 13.5in forward, 0m side, 1.5ft up, tilted back 15 degrees
        Transform3d robotToShooter = new Transform3d(
            new Translation3d(Units.inchesToMeters(13.5), Units.inchesToMeters(0), Units.inchesToMeters(18)),
            new Rotation3d(0, Math.toRadians(-75), 0)
        );
        Pose3d launchPose = m_robotContainer.drivetrain.getPose3d().plus(robotToShooter);

        simJoystick.a().whileTrue(new ShooterSimulationRPMCommand(shooterSimulation));
        m_robotContainer.joystick.b().onTrue(shooterSimulation.launchFuelCommand(launchPose, 28));
    }
}
