// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.shooter.ShootAutomatic;
import frc.robot.commands.shooter.ShootOnTheMoveSequenceCommand;
import frc.robot.commands.shooter.ShootSequence;
import frc.robot.commands.shooter.ShooterFerry;
import frc.robot.commands.Recenter;
import frc.robot.commands.ReverseCommand;
import frc.robot.commands.Hyperjank;
import frc.robot.commands.MoveForwards;
import frc.robot.commands.StopAll;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.RevControllerConstants;
import frc.robot.utils.ShotCalculator;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.limelight.LimelightConstants;
import frc.robot.subsystems.rollers.RollersSubsystem;
import frc.robot.commands.intake.PivotToSetpointCommand;
import frc.robot.commands.intake.RevINeedThis;
import frc.robot.commands.intake.ShootPivot;
import frc.robot.commands.intake.pivotCommand;
import frc.robot.commands.motioncommands.MoveX;
import frc.robot.commands.motioncommands.MoveY;
import frc.robot.commands.motioncommands.RotateTo;
import frc.robot.commands.intake.AutoPivotUp;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeOnTheMove;
import frc.robot.commands.intake.IntakePIDCommand;
import frc.robot.commands.intake.OffsetHome;
import frc.robot.commands.intake.OffsetSetpoint;
import frc.robot.commands.intake.Pivot1;
import frc.robot.commands.intake.Pivot2;
import frc.robot.commands.intake.PivotToHomeCommand;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = -.75 * RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(1);
    private final CommandJoystick buttonboard = new CommandJoystick(0);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final RollersSubsystem rollers = new RollersSubsystem();
    public final Limelight limelight = new Limelight();
    public final Camera camera = new Camera(0, "Camera 1");

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("Shoot",
            drivetrain.applyRequest(
                () -> drive
                    .withRotationalRate(LimelightHelpers.getTX("limelight-two") * LimelightConstants.getAngle((int) LimelightHelpers.getFiducialID("limelight-two")) * .3)
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed * -.8 * .5 * 0)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * -.8 * .5)
            )
            .alongWith(new ShootAutomatic(shooter, rollers))
            .alongWith(new ShootPivot(intake, 1))
        );
        NamedCommands.registerCommand("Pivot Down",
            new PivotToSetpointCommand(intake)
            .withTimeout(1));
        NamedCommands.registerCommand("Field Centric", drivetrain.runOnce(drivetrain::seedFieldCentric));
        NamedCommands.registerCommand("Intake", new IntakeCommand(intake));
        NamedCommands.registerCommand("RecenterToZero", 
            new Recenter(drivetrain, 0)
            .until(() -> Math.abs(drivetrain.getPigeon2().getYaw().getValueAsDouble()) < 5)
            .withTimeout(2));
        NamedCommands.registerCommand("RecenterToShoot", 
           new Hyperjank(drivetrain).withTimeout(1));


        NamedCommands.registerCommand("Move Forward", new MoveForwards(drivetrain, MaxSpeed).withTimeout(3));
        
        // Commands for HyperJank NZ auto
        // as of 3/15/26 in the night, values are merely placeholders
        // .withTimeouts are just here to guarentee command expiration
        // due to large number of namedcommands used in this godforsaken auton,
        // you should keep your timeouts at low values to conserve time
        NamedCommands.registerCommand("SL to CL", 
            new MoveX(drivetrain, 3, MaxSpeed/2)
                .alongWith(new PivotToSetpointCommand(intake))
                .withTimeout(2));
        NamedCommands.registerCommand("Turn around left", 
            new RotateTo(270, MaxAngularRate, drivetrain)
                .withTimeout(2));
        // In PP, Pivot down before calling this
        NamedCommands.registerCommand("Move left and Intake", 
            new MoveY(drivetrain, 1.5, MaxSpeed/3)
                .alongWith(new IntakeCommand(intake))
                .withTimeout(2));
        NamedCommands.registerCommand("Move right", 
            new MoveY(drivetrain, -1.5, MaxSpeed/2)
                .withTimeout(2));
        // Can't schedule two motion commands at the same time because they both use swerve
        NamedCommands.registerCommand("Rotate to 0",
            new RotateTo(0, MaxAngularRate, drivetrain).withTimeout(2));
        NamedCommands.registerCommand("CL To Alli Zone",
            new MoveX(drivetrain, 4.5, MaxSpeed/2)
                .withTimeout(2));
        NamedCommands.registerCommand("Move left to Hub", 
            new MoveY(drivetrain, 2.5, MaxSpeed/2)
                .withTimeout(2));

        // for hyper jank 2
        NamedCommands.registerCommand("SL to CL 2", 
            new MoveX(drivetrain, 3.7, MaxSpeed/2)
                .alongWith(new PivotToSetpointCommand(intake))
                .withTimeout(2));
        
        NamedCommands.registerCommand("Rotate to 180",
            new RotateTo(180, MaxAngularRate, drivetrain).withTimeout(2));
        
        
        autoChooser = AutoBuilder.buildAutoChooser("Hyperjank 2");
        SmartDashboard.putData("Auto Mode", autoChooser);
        camera.streamVideo();

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        // FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        double multiplier = -.8;
        double sotmMultiplier = .5;
        double sotmRotMulti = .05;
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * multiplier) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * multiplier) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * multiplier) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));


        joystick.x().whileTrue(new ReverseCommand(intake, rollers, shooter));
        joystick.y().whileTrue(new ShootSequence(shooter, rollers, 3000));
        joystick.b().whileTrue(
            drivetrain.applyRequest(
                () -> drive
                    .withRotationalRate(LimelightHelpers.getTX("limelight-two") * LimelightConstants.kTrenchAngle * .3)
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed * multiplier * sotmMultiplier * 0)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * multiplier * sotmMultiplier)
            )
            .alongWith(new ShooterFerry(shooter, rollers))
            .alongWith(new ShootPivot(intake, 1))
        ).toggleOnFalse(new PivotToSetpointCommand(intake));

        joystick.povUp().onTrue(new PivotToHomeCommand(intake));
        joystick.povDown().onTrue(new PivotToSetpointCommand(intake));
        joystick.povLeft().whileTrue(new pivotCommand(intake, .09));
        joystick.povRight().whileTrue(new pivotCommand(intake, -.09));
        
        joystick.button(5).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        
        joystick.axisGreaterThan(2, .3).whileTrue(
            drivetrain.applyRequest(
                () -> drive
                    .withRotationalRate(LimelightHelpers.getTX("limelight-two") * LimelightConstants.getAngle((int) LimelightHelpers.getFiducialID("limelight-two")) * .3)
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed * multiplier * sotmMultiplier * 0)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * multiplier * sotmMultiplier)
            )
            .alongWith(new ShootAutomatic(shooter, rollers))
            .alongWith(new ShootPivot(intake, 1))
        ).toggleOnFalse(new PivotToSetpointCommand(intake));
        joystick.axisGreaterThan(3, .3).whileTrue(
            drivetrain.applyRequest(
                () -> drive
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * multiplier)
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed * multiplier * .5)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * multiplier * .5)
            )
            .alongWith(new IntakeCommand(intake))
        );

        buttonboard.button(11).onTrue(new StopAll(intake, shooter, rollers));
        buttonboard.button(9).onTrue(new OffsetHome(intake));
        buttonboard.button(2).onTrue(new OffsetSetpoint(intake));
        buttonboard.button(6).onTrue(new PivotToHomeCommand(intake));
        buttonboard.button(7).onTrue(new PivotToSetpointCommand(intake));
        // buttonboard.button(3).whileTrue(drivetrain.applyRequest(
        //         () -> drive
        //             .withRotationalRate(LimelightHelpers.getTX("limelight") * -LimelightConstants.kHubAngle * 0.15)
        //             .withVelocityX(-joystick.getLeftY() * MaxSpeed * multiplier * sotmMultiplier * 0)
        //             .withVelocityY(-joystick.getLeftX() * MaxSpeed * multiplier * sotmMultiplier)
        //     )
        //     .alongWith(new ShootAutomatic(shooter, rollers))
        //     .alongWith(new ShootPivot(intake, 1))
        // ).toggleOnFalse(new PivotToSetpointCommand(intake));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // if PathPlanner is giving issues try manually doing the auto here
        // 99 percent of the auto is just calling motion commands one after the other
        // so re-writing it here should be easy. 2024 2 note auto moment

        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
        }
}
