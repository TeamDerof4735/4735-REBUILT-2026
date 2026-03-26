// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.teleop.ShooterTests;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class auto1 extends SequentialCommandGroup {
  PathPlannerAuto auto1 = new PathPlannerAuto("auto 1");
  ShooterSubsystem shooter;
  ConveyorSubsystem conveyor;

  public auto1(ShooterSubsystem shooter, ConveyorSubsystem conveyor) {
    this.shooter = shooter;
    this.conveyor = conveyor;
    addRequirements(shooter, conveyor);

    
    addCommands(
      auto1,
      new ShooterTests(shooter, conveyor, 2050).withTimeout(3.5)
    );
  }
}
