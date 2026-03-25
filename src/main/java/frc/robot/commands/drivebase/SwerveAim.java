
package frc.robot.commands.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;


public class SwerveAim extends Command {

  SwerveSubsystem swerveSubsystem;
  private DoubleSupplier translationX;
  private DoubleSupplier translationY;
  private final ProfiledPIDController rotationController;
  
  public SwerveAim(SwerveSubsystem SwerveSubsystem, DoubleSupplier translationX, DoubleSupplier translationY) {

    this.swerveSubsystem = SwerveSubsystem;

    this.rotationController =
        new ProfiledPIDController(
          3.5,
          0,
          0.25,
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(360), Units.degreesToRadians(360)));

    this.rotationController.setIZone(0);
    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
    this.rotationController.setTolerance(0.5);
    this.translationX = translationX;
    this.translationY = translationY;

    addRequirements(swerveSubsystem);

  }

  @Override
  public void initialize() {}

  
  @Override
  public void execute() {

    double xSpeed = Math.pow(translationX.getAsDouble(), 5)
    * swerveSubsystem.getSwerve().getMaximumChassisVelocity();

    double ySpeed = Math.pow(translationY.getAsDouble(), 5)
    * swerveSubsystem.getSwerve().getMaximumChassisVelocity();

    double rotationOutput = rotationController.calculate(swerveSubsystem.ActAngle(), swerveSubsystem.error());

    swerveSubsystem.drive(
      new Translation2d(xSpeed, ySpeed), 
      rotationOutput, 
      true);
  }

  
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return rotationController.atSetpoint();
  }
  
}
