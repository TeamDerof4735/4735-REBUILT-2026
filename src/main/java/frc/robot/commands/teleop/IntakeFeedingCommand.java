package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.wristConstants;
import frc.robot.subsystems.WristSubystem;

public class IntakeFeedingCommand extends SequentialCommandGroup {

  public IntakeFeedingCommand(WristSubystem wristSubystem) {

    addRequirements(wristSubystem);

    addCommands(
        new WristRunPos(wristSubystem, 0, wristConstants.kP, wristConstants.kI, wristConstants.kD)
            .withTimeout(1),
        new WaitCommand(0.2),
        new WristRunPos(wristSubystem, 0, wristConstants.kP, wristConstants.kI, wristConstants.kD)
            .withTimeout(1)
    );
  }
}