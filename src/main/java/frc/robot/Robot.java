// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    private boolean kUseLimelight = false;
    
        // // Simulation
        // private ShooterSimulation shooterSimulation = new ShooterSimulation();
        // private CommandXboxController simJoystick = new CommandXboxController(3);
        // private IntakeSimulation intakeSimulation = new IntakeSimulation();
    
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
            SmartDashboard.putNumber("Drivetrain Speed", m_robotContainer.drivetrain.getState().Speeds.vxMetersPerSecond);
            SmartDashboard.putBoolean("Active Hub", isHubActive());
            SmartDashboard.putString("Message", DriverStation.getGameSpecificMessage());
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
    
                LimelightHelpers.SetRobotOrientation("limelight-two", headingDeg, 0, 0, 0, 0, 0);
                var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-two");
                if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
                    m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
                }
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
            // set yaw could cause problem for hyperjank 2 auto (I think)?
            // m_robotContainer.drivetrain.getPigeon2().setYaw(0);
    
            // make sure robot doesn't start moving in the opposite direction upon power-on
            // once auton is finished, power cycle robot and run auton, then redeploy code and run auton again
            // m_robotContainer.drivetrain.seedFieldCentric();
            kUseLimelight = true;
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
        kUseLimelight = false;
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
        // simJoystick.a().whileTrue(new ShooterSimulationRPMCommand(shooterSimulation));
        // simJoystick.x().onTrue(new ShooterSimDistanceSetter(shooterSimulation, 66));
        // simJoystick.y().onTrue(new ShooterSimDistanceSetter(shooterSimulation, 102));
        // simJoystick.b().onTrue(new ShooterSimDistanceSetter(shooterSimulation, 140));
        // simJoystick.a().onTrue(new IntakeSimPIDCommand(intakeSimulation, 110, .0175));
        // simJoystick.b().onTrue(new IntakeSimPIDCommand(intakeSimulation, 330, .0175));
    }

    public boolean isHubActive() {
        
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
            // If we have invalid game data, assume hub is active.
            return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }
}
