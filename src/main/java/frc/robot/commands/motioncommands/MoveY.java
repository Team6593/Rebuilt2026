// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.motioncommands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class MoveY extends Command {

    private final CommandSwerveDrivetrain drivebase;
    private final double yAmount;

    private double startY;
    private final double maxSpeed;

    // This is going to be used in autons so I'm using FieldCentric for sake of ease.
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public MoveY(CommandSwerveDrivetrain drivebase, double yAmount, double maxSpeed) {
        this.drivebase = drivebase;
        this.yAmount = yAmount;
        this.maxSpeed = maxSpeed;
    }

    @Override
    public void initialize() {
        startY = drivebase.getState().Pose.getY();
    }

    @Override
    public void execute() {
        double direction = Math.signum(yAmount);

        drivebase.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(direction * maxSpeed)
                .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        double currentY = drivebase.getState().Pose.getY();
        return Math.abs(currentY - startY) >= Math.abs(yAmount);
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