// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.motioncommands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class MoveXPro extends Command {

    private final CommandSwerveDrivetrain drivebase;
    private final double xAmount;
    private final double maxSpeed;

    private final double kP = 0.2;
    private final double tolerance = 0.02;

    private double startX;
    private double targetX;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public MoveXPro(CommandSwerveDrivetrain drivebase, double xAmount, double maxSpeed) {
        this.drivebase = drivebase;
        this.xAmount = xAmount;
        this.maxSpeed = maxSpeed;

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        startX = drivebase.getState().Pose.getX();
        targetX = startX + xAmount;
    }

    @Override
    public void execute() {
        double currentX = drivebase.getState().Pose.getX();
        double error = targetX - currentX;

        double velocity = MathUtil.clamp(kP * error, -maxSpeed, maxSpeed);

        drivebase.setControl(
            driveRequest
                .withVelocityX(velocity)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        double currentX = drivebase.getState().Pose.getX();
        return Math.abs(targetX - currentX) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }
}