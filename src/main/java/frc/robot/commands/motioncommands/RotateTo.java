// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.motioncommands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RotateTo extends Command {

    private final CommandSwerveDrivetrain drivebase;
    private final double targetAngle;
    private final double maxAngularRotation;

    private final PIDController rotationPID = new PIDController(0.5, 0, 0);

    // This is going to be used in autons so I'm using FieldCentric for sake of ease.
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public RotateTo(double angleDegrees, double maxAngularRotation, CommandSwerveDrivetrain drivebase) {
        this.drivebase = drivebase;
        this.targetAngle = angleDegrees;
        this.maxAngularRotation = maxAngularRotation;

        rotationPID.enableContinuousInput(-180, 180);
        rotationPID.setTolerance(2.0);
    }

    @Override
    public void initialize() {
        rotationPID.setSetpoint(targetAngle);
    }

    @Override
    public void execute() {
        System.out.println("ROTATING");
        double currentHeading = drivebase.getState().Pose.getRotation().getDegrees();

        double output = rotationPID.calculate(currentHeading);
        output = MathUtil.clamp(output, -maxAngularRotation, maxAngularRotation);

        drivebase.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(output)
        );
    }

    @Override
    public boolean isFinished() {
        return rotationPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
      System.out.println("ROTATING ENDED!!!!!!!!!!!");
        drivebase.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }
}