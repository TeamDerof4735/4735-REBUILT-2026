package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import swervelib.SwerveDrive;


public class Vision extends SubsystemBase
{

  private final SwerveDrive         swerveDrive;

  public Pose2d finalPose;

  private final SwerveSubsystem swerveSubsystem;

  public boolean useVision = true;
    private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

    // Subsystem & Limelight
    
    private final String limelightNames = "limelight-derof";

    // Pose estimation
    private double averageTagDistance = 0.0;
    private PoseEstimate lastAcceptedVision = null;

    //Logging
    List<Pose3d> allTagPoses = new ArrayList<>();

  public Vision (SwerveSubsystem swerveSubsystem) {

    this.swerveSubsystem = swerveSubsystem;

    swerveDrive = swerveSubsystem.getSwerveDrive();

  }

  public void periodic () {

    LimelightHelpers.SetRobotOrientation(
            limelightNames, 
            swerveDrive.getYaw().getDegrees(), 
            0, 0, 0, 0, 0
    );

    PoseEstimate mt2Estimate = mt2(isRedAlliance());

    if (mt2Estimate == null || mt2Estimate.tagCount == 0) {
        averageTagDistance = 0;
        return;
    }

    averageTagDistance = mt2Estimate.avgTagDist;

    // Calculate standard deviations
        double xyStdDev = VisionConstants.xyStdDevCoefficient 
                        * Math.pow(averageTagDistance, 2) 
        / ((LimelightHelpers.getTargetCount(limelightNames) == 0) ? 100 : LimelightHelpers.getTargetCount(limelightNames));

        lastTagDetectionTimes.put((int)LimelightHelpers.getFiducialID(limelightNames), Timer.getTimestamp());

        // Apply measurement standard deviations
        swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, 9999999));

        // Process odometry and log vision data
        swerveSubsystem.filterOutOfFieldData();
        odometryWithVision(mt2Estimate);

        
  }

  public void odometryWithVision(PoseEstimate newMt) {

    if (!LimelightHelpers.getTV(limelightNames) || newMt == null || newMt.tagCount == 0) {
        return;
    }

    Pose2d currentPose = swerveSubsystem.getPose();

    double error = currentPose.getTranslation()
      .getDistance(newMt.pose.getTranslation());

    if (error > 1.0) {
      return;
    }

    if (Math.abs(swerveSubsystem.velAng()) > 720) {
      return;
    }

    if (linearSpeedMps() > 3.0) {
      return;
    }

    boolean outOfBounds = 
              (newMt.pose.getX() < -Constants.fieldBorderMargin)
          || (newMt.pose.getX() > Constants.fieldLength + Constants.fieldBorderMargin)
          || (newMt.pose.getY() < -Constants.fieldBorderMargin)
          || (newMt.pose.getY() > Constants.fieldWidth + Constants.fieldBorderMargin);
            
    if (outOfBounds || newMt.tagCount == 0) {
      return;
    }

    if (isBigJump(swerveSubsystem.getPose()) && newMt.tagCount == 1) {
      return;
    }

    swerveDrive.addVisionMeasurement(newMt.pose, newMt.timestampSeconds);
    lastAcceptedVision = newMt;
  }




  public PoseEstimate mt2(boolean allianceRed) {
    return allianceRed
        ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightNames)
        : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightNames);
  } 

  public double linearSpeedMps() {
    ChassisSpeeds robotVelocity = swerveDrive.getRobotVelocity();
    return Math.hypot(
        robotVelocity.vxMetersPerSecond,
        robotVelocity.vyMetersPerSecond
    );
  } 

  private boolean isBigJump(Pose2d newMt) {
    if (newMt == null || lastAcceptedVision == null) {
        return false;
    }

    double dx = newMt.getX()- lastAcceptedVision.pose.getX();
    double dy = newMt.getY() - lastAcceptedVision.pose.getY();
    double distance = Math.hypot(dx, dy);

    return distance > 1.5; // empieza con 1 metro, luego ajustas
  }

  private boolean isRedAlliance() {
  return DriverStation.getAlliance().isPresent()
      && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }

}


