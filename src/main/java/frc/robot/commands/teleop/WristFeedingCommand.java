package frc.robot.commands.teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubystem;
import frc.robot.Constants.wristConstants;

public class WristFeedingCommand extends Command {

  private final WristSubystem wrist;
  private final PIDController pid;

  private final double posA = 0.32;
  private final double posB = 0.29;

  private double currentSetpoint;
  private boolean goingToB = true;

  public WristFeedingCommand(WristSubystem wrist) {
    this.wrist = wrist;

    this.pid = new PIDController(
        wristConstants.kP,
        wristConstants.kI,
        wristConstants.kD
    );

    addRequirements(wrist);
  }

  @Override
  public void initialize() {
    currentSetpoint = posA;
    pid.reset();
    pid.setSetpoint(currentSetpoint);
  }

  @Override
  public void execute() {

    double output = pid.calculate(wrist.getEnoderPosition());
    wrist.wristRun(output);

    if (pid.atSetpoint()) {

      if (goingToB) {
        currentSetpoint = posB;
      } else {
        currentSetpoint = posA;
      }

      goingToB = !goingToB;
      pid.setSetpoint(currentSetpoint);
    }
  }

  @Override
  public void end(boolean interrupted) {
    wrist.wristRun(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}