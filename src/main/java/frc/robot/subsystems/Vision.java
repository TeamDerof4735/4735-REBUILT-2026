package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Robot;

import java.awt.Desktop;
import java.time.Period;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;


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


