// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.motioncommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class MoveYPro extends Command {

    private final CommandSwerveDrivetrain drivebase;
    private final double yAmount;
    private final double maxSpeed;

    private final double kP = 0.2;
    private final double tolerance = 0.02;

    private double startY;
    private double targetY;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public MoveYPro(CommandSwerveDrivetrain drivebase, double yAmount, double maxSpeed) {
        this.drivebase = drivebase;
        this.yAmount = yAmount;
        this.maxSpeed = maxSpeed;

        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        startY = drivebase.getState().Pose.getY();
        targetY = startY + yAmount;
    }

    @Override
    public void execute() {
        double currentY = drivebase.getState().Pose.getY();
        double error = targetY - currentY;

        double velocity = MathUtil.clamp(kP * error, -maxSpeed, maxSpeed);

        drivebase.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(velocity)
                .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        double currentY = drivebase.getState().Pose.getY();
        return Math.abs(targetY - currentY) < tolerance;
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