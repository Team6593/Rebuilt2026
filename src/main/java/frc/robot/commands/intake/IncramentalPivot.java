// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.utils.ShotCalculator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IncramentalPivot extends Command {

  IntakeSubsystem intake;
  double stage;

  /** Creates a new IncramentalPivot. */
  public IncramentalPivot(IntakeSubsystem intake) {
    this.intake = intake;
    
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setStage(1);
    System.out.println("INIT!!");
  }

  // // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
  //   if (((intake.stage() % 2) != 0)) {
  //     System.out.println(intake.getPosition() > 40 + (intake.stage() * 40));
  //     if (intake.getPosition() > 40 + (intake.stage() * 40)) {
  //       intake.changeStage(1);
  //     } else {
  //       intake.pivot(.2);
  //     }
  //   } else if (((stage % 2) == 0)) {
  //     System.out.println(intake.getPosition() > 40 + (intake.stage() * 40));
  //     if (intake.getPosition() < 40) {
  //       intake.pivot(-.15);
  //     } else {
  //       intake.changeStage(1);
  //     }
  //   } 
  //   if (stage > 5) {
  //     intake.changeStage(-1);
  //   }
  // }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Store the current stage in a local variable for cleaner code
    int currentStage = intake.stage(); 
    if (SmartDashboard.getNumber("ShooterM RPM", 0) > ShotCalculator.lerpGet(SmartDashboard.getNumber("Last Distance", 0)).rpm) {
      if ((currentStage % 2) != 0) {
        // ===== ODD STAGES: GOING UP =====
        double targetUpPosition = 40 + (currentStage * 40);
        if (targetUpPosition > 220) {
          targetUpPosition = 220;
        }
        
        // If we haven't reached the target, keep going up
        if (intake.getPosition() < targetUpPosition) {
          intake.pivot(0.2);
        } else {
          // Target reached, move to the next stage
          intake.changeStage(1);
        }

      } else {
        // ===== EVEN STAGES: GOING DOWN =====
        
        // If we are currently above 40, keep going down
        if (intake.getPosition() > 40) {
          intake.pivot(-0.15);
        } else {
          // Reached the bottom target, move to the next stage
          intake.changeStage(1);
        }
      }
    }  

    // Note on your previous cap logic:
    // If you want the sequence to completely stop after a certain stage,
    // you should handle that in your isFinished() method rather than 
    // constantly subtracting 1 from the stage here.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    System.out.println("END!!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
