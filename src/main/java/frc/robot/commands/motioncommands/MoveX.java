// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.motioncommands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class MoveX extends Command {

    private final CommandSwerveDrivetrain drivebase;
    private final double xAmount;

    private double startX;
    private final double maxSpeed;

    // This is going to be used in autons so I'm using FieldCentric for sake of ease.
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public MoveX(CommandSwerveDrivetrain drivebase, double xAmount, double maxSpeed) {
        this.drivebase = drivebase;
        this.xAmount = xAmount;
        this.maxSpeed = maxSpeed;
    }

    @Override
    public void initialize() {
        startX = drivebase.getState().Pose.getX();
    }

    @Override
    public void execute() {
        double direction = Math.signum(xAmount);

        drivebase.setControl(
            driveRequest
                .withVelocityX(direction * maxSpeed)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        double currentX = drivebase.getState().Pose.getX();
        return Math.abs(currentX - startX) >= Math.abs(xAmount);
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