package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterInterpolation;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;

public class ShooterCommand extends Command {

  private final ShooterSubsystem shooter;
  private final ConveyorSubsystem conveyor;
  private final ShooterInterpolation shooterInterpolation;
  private final WristFeedingCommand wristCommand;
  private final Vision vision;

  private double distanceMeters;
  private double targetRPM;

  private final Timer timer = new Timer();

  private boolean wristActivated = false;

  public ShooterCommand(
      ShooterSubsystem shooter,
      ConveyorSubsystem conveyor,
      ShooterInterpolation shooterInterpolation,
      WristFeedingCommand wristCommand, 
      Vision vision) {

    this.shooter = shooter;
    this.conveyor = conveyor;
    this.shooterInterpolation = shooterInterpolation;
    this.wristCommand = wristCommand;
    this.vision = vision;

    addRequirements(shooter, conveyor, vision);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.stop();
    wristActivated = false;
  }

  @Override
  public void execute() {


    distanceMeters = vision.hubDis();

    // RPM
    targetRPM = shooterInterpolation.calculateRPM(distanceMeters);
    shooter.setShooterRPM(targetRPM);

    if (!wristActivated && shooter.getCurrentRPM() >= (targetRPM - 50)) {
      wristActivated = true;

      //CommandScheduler.getInstance().schedule(wristCommand);
    }

    if (shooter.getCurrentRPM() >= (targetRPM - 50)) {

      if (!timer.isRunning()) {
        timer.restart();
      }

      if (timer.get() >= 0.125) {
        shooter.indexMove(-0.8);
        conveyor.conveyorMove(0.9);
      } else {
        shooter.indexMove(0);
        conveyor.conveyorMove(0);
      }

    } else {
      shooter.indexMove(0);
      conveyor.conveyorMove(0);

      timer.stop();
      timer.reset();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setShooterRPM(0);
    shooter.indexMove(0);
    conveyor.conveyorMove(0);

    if (wristCommand.isScheduled()) {
      wristCommand.cancel();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}