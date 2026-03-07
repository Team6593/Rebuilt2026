// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.shooter.ShootOnTheMoveSequenceCommand;
import frc.robot.commands.shooter.ShootSequence;
import frc.robot.commands.ReverseCommand;
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
import frc.robot.commands.intake.ShootPivot;
import frc.robot.commands.intake.pivotCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeOnTheMove;
import frc.robot.commands.intake.IntakePIDCommand;
import frc.robot.commands.intake.PivotToHomeCommand;

public class RobotContainer {
    private double MaxSpeed = -1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final RollersSubsystem rollers = new RollersSubsystem();
    public final Limelight limelight = new Limelight();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        // FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        double multiplier = -.5;
        double sotmMultiplier = .5;
        double sotmRotMulti = .05;
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * multiplier) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * multiplier) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * multiplier * 2) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // joystick.povUp().whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        // joystick.povDown().whileTrue(drivetrain.applyRequest(() ->

        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // joystick.button(RevControllerConstants.m_M2).whileTrue(new ShootSequence(shooter, intake, feeder));
        // joystick.button(RevControllerConstants.m_square).onTrue(new StopAll(feeder, intake, shooter));
        // joystick.y().onTrue(new PivotToHomeCommand(intake));
        // joystick.b().onTrue(new PivotToSetpointCommand(intake));
        // joystick.button(RevControllerConstants.m_share).onTrue(new PivotToHomeCommand(intake));
        // joystick.button(RevControllerConstants.m_options).onTrue(new PivotToSetpointCommand(intake));
        // joystick.button(RevControllerConstants.m_M1).whileTrue(new IntakeCommand(intake));

        // joystick.b().whileTrue(new IntakeCommand(intake));
        // joystick.button(6).whileTrue(new IntakeCommand(intake));
        joystick.a().whileTrue(new ReverseCommand(intake, rollers, shooter));
        joystick.y().whileTrue(new ShootSequence(shooter, intake, rollers, 2425));
        // joystick.x().whileTrue(new ShootSequence(shooter, rollersSubsystem, 1450));
        // joystick.button(7).onTrue(new PivotToHomeCommand(intake));
        // joystick.button(8).onTrue(new PivotToSetpointCommand(intake));
        // joystick.y().whileTrue(new ShootSequence(shooter, intake, rollersSubsystem, 6000));
        joystick.povUp().onTrue(new PivotToHomeCommand(intake));
        joystick.povDown().onTrue(new PivotToSetpointCommand(intake));


        // Reset the field-centric heading on pleft bumper press.
        joystick.button(5).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        // joystick.axisGreaterThan(2, .3).whileTrue(
        //     drivetrain.applyRequest(
        //         () -> drive
        //             .withRotationalRate(LimelightHelpers.getTX("limelight") * (LimelightConstants.kHubAngle * sotmRotMulti))
        //             .withVelocityX(-joystick.getLeftY() * MaxSpeed * multiplier * sotmMultiplier * lockedMultiplier)
        //             .withVelocityY(-joystick.getLeftX() * MaxSpeed * multiplier * sotmMultiplier)
        //     )
        //     .alongWith(new ShootSequence(shooter, rollers, ShotCalculator.lerpGet(limelight.estimateDistance()).rpm - 25))
        //     .alongWith(new ShootPivot(intake, .0175))
        // ).toggleOnFalse(new PivotToSetpointCommand(intake));
        joystick.axisGreaterThan(2, .3).whileTrue(
            drivetrain.applyRequest(
                () -> drive
                    .withRotationalRate(LimelightHelpers.getTX("limelight") * (LimelightConstants.kHubAngle * sotmRotMulti))
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed * multiplier * .1)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * multiplier * sotmMultiplier * 0)
            )
            // .alongWith(new ShootSequence(shooter, intake, rollers, ShotCalculator.lerpGet(limelight.estimateDistance()).rpm + 25))
            .alongWith(new ShootSequence(shooter, intake, rollers, ShotCalculator.lerpGet(LimelightHelpers.getTargetPose_RobotSpace("limelight")[2] *39.37).rpm + 25)
        ));
        // joystick.axisGreaterThan(3, .3).whileTrue(new IntakeCommand(intake));

        
        joystick.axisGreaterThan(3, .3).whileTrue(
            drivetrain.applyRequest(
                () -> drive
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * multiplier * 2)
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed * multiplier * .5)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * multiplier * .5)
            )
            .alongWith(new IntakeCommand(intake))
        );


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
