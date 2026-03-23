package frc.robot.commands.drivebase;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import frc.robot.subsystems.SwerveSubsystem;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;



public class SwerveAutoAlignPose extends Command {
  private final Supplier<Pose2d>  redPose;
  private final Supplier<Pose2d> bluePose;
  private Supplier<Pose2d> targetPose;
  private double invertVal = 1.0;
  


  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController rotationController;

  private double MaxSpeed = Constants.MAX_SPEED;

  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); 
          
  private final SwerveRequest.FieldCentric fieldCentricdrive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1)
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage);

  private SwerveSubsystem m_swerve;

  public SwerveAutoAlignPose(Supplier<Pose2d> redPose, Supplier<Pose2d> bluePose, SwerveSubsystem m_swerve) {

    this.xController =
        new ProfiledPIDController(
            30,
            0,
            0,
            new TrapezoidProfile.Constraints(3.0, 2.0));
    this.yController =
        new ProfiledPIDController(
          30,
          0,
          0,
            new TrapezoidProfile.Constraints(3.0, 2.0));
    this.rotationController =
        new ProfiledPIDController(
          20,
          0,
          0,
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(360), Units.degreesToRadians(360)));

    this.yController.setIZone(1.254);
    this.xController.setIZone(1.254);

    this.rotationController.setIZone(0);
    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

    this.m_swerve = m_swerve;
    this.redPose = redPose;
    this.bluePose = bluePose;  
    addRequirements(m_swerve);
  }

  @Override
  public void initialize() {

    if (m_swerve.isRedAlliance() == true) {
      targetPose = redPose;
      invertVal = -1.0;
    } else {
      targetPose = bluePose;
      invertVal = 1.0;
    }


    Pose2d currentPose = m_swerve.getPose();
        ChassisSpeeds currentSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            m_swerve.getRobotVelocity(), currentPose.getRotation());

    xController.reset(currentPose.getX(), currentSpeeds.vxMetersPerSecond);
    yController.reset(currentPose.getY(), currentSpeeds.vyMetersPerSecond);
    rotationController.reset(
        currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void execute() {
    Pose2d currentPose = m_swerve.getPose();

    double xFeedback = xController.calculate(currentPose.getX(), targetPose.get().getX());
    double yFeedback = yController.calculate(currentPose.getY(), targetPose.get().getY());
    double rotFeedback =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), targetPose.get().getRotation().getRadians());

    double xFF = xController.getSetpoint().velocity;
    double yFF = yController.getSetpoint().velocity;
    double rotFF = rotationController.getSetpoint().velocity;

    double xVel = xFF + xFeedback;
    double yVel = yFF + yFeedback;
    double rotVel = rotFF + rotFeedback;

    if (Math.abs(currentPose.getX() - targetPose.get().getX()) < 0.004) {
      xVel = 0;
    }
    if (Math.abs(currentPose.getY() - targetPose.get().getY()) < 0.004) {
      yVel = 0;
    }
    if (Math.abs(currentPose.getRotation().minus(targetPose.get().getRotation()).getDegrees()) < 0.01) {
      rotVel = 0;
    }

    m_swerve.drive(
    new Translation2d(xVel * invertVal, yVel * invertVal), 
            -rotVel,                                               
    true                                                   
    );


    SmartDashboard.setDefaultNumber("Swerve/PIDOutput X",  xVel);
    SmartDashboard.setDefaultNumber("Swerve/PIDOutput Y",  yVel);
    SmartDashboard.setDefaultNumber("Swerve/PIDOutput T",  rotVel);  
    SmartDashboard.setDefaultNumber("Swerve/Invert Val", invertVal);
    }

  @Override
  public boolean isFinished() {
    return false;
  }
}