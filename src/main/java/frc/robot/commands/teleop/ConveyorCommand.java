// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeandIndex;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IndexerCom extends Command {
  
  IntakeandIndex intakeandIndex;

  public IndexerCom(IntakeandIndex intakeandIndex) {
    this.intakeandIndex = intakeandIndex;
    addRequirements(intakeandIndex);
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    intakeandIndex.runINTandIND(1);
  }

  
  @Override
  public void end(boolean interrupted) {
    intakeandIndex.runINTandIND(0);
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
