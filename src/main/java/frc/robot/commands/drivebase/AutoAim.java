
package frc.robot.commands.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import java.util.function.DoubleSupplier;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAim extends Command {

  SwerveSubsystem swerveSubsystem;
  private DoubleSupplier translationX;
  private DoubleSupplier translationY;
  private final PIDController rotationController;
  
  public AutoAim(SwerveSubsystem swerveSubsystem, DoubleSupplier translationX, DoubleSupplier translationY) {

    this.rotationController = 
      new PIDController(
        0, 
        0, 
        0);


    rotationController.setTolerance(0.5);

    this.swerveSubsystem = swerveSubsystem;
    this.translationX = translationX;
    this.translationY = translationY;
    addRequirements(swerveSubsystem);

   
  }

  
  @Override
  public void initialize() {
    
  }

  
  @Override
public void execute() {
    double xSpeed = Math.pow(translationX.getAsDouble(), 5)
        * swerveSubsystem.getSwerve().getMaximumChassisVelocity();

    double ySpeed = Math.pow(translationY.getAsDouble(), 5)
        * swerveSubsystem.getSwerve().getMaximumChassisVelocity();

    double rotationOutput = 0.0;

    if (LimelightHelpers.getTV("limelight-derof")) {

        double limelightTx = LimelightHelpers.getTX("limelight-derof");
        rotationOutput = rotationController.calculate(limelightTx, 0.0);
        
    } 

    swerveSubsystem.drive(
      new Translation2d(xSpeed, ySpeed), 
      rotationOutput, 
      false);

}

  
  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return rotationController.atSetpoint();
  }
}
