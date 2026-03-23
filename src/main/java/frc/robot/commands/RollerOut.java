

package frc.robot.commands;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.ShooterLookup;
import frc.robot.subsystems.IntakeandIndex;
import frc.robot.subsystems.shooter;


public class RollerOut extends Command {

  shooter shooter;
  ShooterLookup shooterLookup;
  double distanceMeters;
  private final Timer Timer = new Timer();

  
  
  public RollerOut(shooter shooter, ShooterLookup shooterLookup) {
    this.shooter = shooter;
    this.shooterLookup = shooterLookup;
    addRequirements(shooter);
  }

  
  @Override
  public void initialize() {
    Timer.reset();
  }

  
  @Override
  public void execute() {
    // Obtener pose del AprilTag en espacio de cámara
    double[] pose = LimelightHelpers.getTargetPose_CameraSpace("limelight-derof");

    if (pose != null && pose.length >= 3 && LimelightHelpers.getTV("limelight-derof")) {
        distanceMeters = pose[2];
    } else {
      distanceMeters = 0;
    }
    
    double potencia = shooterLookup.getRPM(distanceMeters);

    shooter.shootVel(potencia);
    if (shooter.left_rpm() >= potencia){
      Timer.start();
      if (Timer.get() >= 0.2){
        shooter.shootVel(potencia);
        shooter.conveyorPOT(-0.7);
      } else {
        shooter.shootVel(potencia);
      }
    } else {
      shooter.shootVel(potencia);
      shooter.conveyorPOT(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.shooterRun(0);
    shooter.conveyorPOT(0);
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
