package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterInterpolation;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {

  private final ShooterSubsystem shooter;
  private final ConveyorSubsystem conveyor;
  private final ShooterInterpolation shooterInterpolation;
  private final IntakeFeedingCommand intakeCommand;

  private double distanceMeters;
  private double targetRPM;

  private final Timer timer = new Timer();
  private boolean intakeStarted = false;

  public ShooterCommand(
      ShooterSubsystem shooter,
      ConveyorSubsystem conveyor,
      ShooterInterpolation shooterInterpolation,
      IntakeFeedingCommand intakeCommand) {

    this.shooter = shooter;
    this.conveyor = conveyor;
    this.shooterInterpolation = shooterInterpolation;
    this.intakeCommand = intakeCommand;

    addRequirements(shooter, conveyor);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.stop();
    intakeStarted = false;
  }

  @Override
  public void execute() {

    // Distancia con Limelight
    double[] pose = LimelightHelpers.getTargetPose_CameraSpace("limelight-derof");

    if (pose != null && pose.length >= 3 && LimelightHelpers.getTV("limelight-derof")) {
      distanceMeters = pose[2];
    } else {
      distanceMeters = 0;
    }

    targetRPM = shooterInterpolation.calculateRPM(distanceMeters);

    shooter.shooterSpeed(targetRPM);

    if (shooter.getShooterRPM() >= (targetRPM - 20)) { 

      if (!timer.isRunning()) {
        timer.restart();
      }

      if (timer.get() >= 0.4) {

        shooter.indexMove(-0.7);
        conveyor.conveyorMove(0.7);

        if (!intakeStarted) {
          intakeCommand.schedule();
          intakeStarted = true;
        }

      } else {
        shooter.indexMove(0);
        conveyor.conveyorMove(0);
      }

    } else {
      shooter.indexMove(0);
      conveyor.conveyorMove(0);
      timer.stop();
      timer.reset();
      intakeStarted = false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.shooterSpeed(0);
    shooter.indexMove(0);
    conveyor.conveyorMove(0);

    if (intakeCommand.isScheduled()) {
      intakeCommand.cancel();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}