package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  
  ShooterInterpolation shooterInterpolation;

  public Pose2d finalPose;

  private final SwerveSubsystem swerveSubsystem;

  public boolean useVision = true;
    private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

    // Subsystem & Limelight
    
    private final String limelightNames = "limelight-derof";

    // Pose estimation
    private PoseEstimate lastAcceptedVision = null;

    private double averageTagDistance = 0.0;
    private static PoseEstimate mt2;
    private static PoseEstimate oldMt = new PoseEstimate();

    //Logging
    List<Pose3d> allTagPoses = new ArrayList<>();

  public Vision (SwerveSubsystem swerveSubsystem, ShooterInterpolation shooterInterpolation) {

    this.swerveSubsystem = swerveSubsystem;
    this.shooterInterpolation = shooterInterpolation;
    
    swerveDrive = swerveSubsystem.getSwerveDrive();

  }

  public void periodic () {

    SmartDashboard.putNumber("rpm requeridas", shooterInterpolation.calculateRPM(hubDis()));
    SmartDashboard.putNumber("distancia", hubDis());

    LimelightHelpers.SetRobotOrientation(
            limelightNames, 
            swerveDrive.getYaw().getDegrees(), 
            0, 0, 0, 0, 0
    );

    averageTagDistance = (mt2 == null) ? 0 : mt2.avgTagDist;

    // Calculate standard deviations
        double xyStdDev = VisionConstants.xyStdDevCoefficient 
                        * Math.pow(averageTagDistance, 2) 
        / ((LimelightHelpers.getTargetCount(limelightNames) == 0) ? 100 : LimelightHelpers.getTargetCount(limelightNames));

        lastTagDetectionTimes.put((int)LimelightHelpers.getFiducialID(limelightNames), Timer.getTimestamp());

        // Apply measurement standard deviations
        swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, 9999999));

        // Process odometry and log vision data
        swerveSubsystem.filterOutOfFieldData();
        odometryWithVision(limelightNames, xyStdDev,0);

        
  }

  public void odometryWithVision(String limelightName, double xySTD, double thetaSTD) {
    if (LimelightHelpers.getTV(limelightNames) && LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightNames) != null) {
        
        if (mt2 != null) {
            oldMt = mt2;
        }

        // Fetch latest bot pose
        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightNames) == null ? new PoseEstimate():LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightNames) ;
        boolean doRejectUpdate = false;

        // Reject updates if robot is spinning too fast
        if (Math.abs(swerveSubsystem.velAng()) > 720) {
            doRejectUpdate = true;
        }

        // Reject updates if pose is out of field bounds
        boolean outOfBounds = 
               (mt2.pose.getX() < -Constants.fieldBorderMargin)
            || (mt2.pose.getX() > Constants.fieldLength + Constants.fieldBorderMargin)
            || (mt2.pose.getY() < -Constants.fieldBorderMargin)
            || (mt2.pose.getY() > Constants.fieldWidth + Constants.fieldBorderMargin);
            
        if (outOfBounds || mt2.tagCount == 0) {
            doRejectUpdate = true;
        }

        // Reject updates if there's a sudden position jump
        boolean suddenJump = 
               (Math.abs(oldMt.pose.getX() - mt2.pose.getX()) > 0.2)
            || (Math.abs(oldMt.pose.getY() - mt2.pose.getY()) > 0.2);

        if (suddenJump) {
            doRejectUpdate = true;
        }


        if (linearSpeedMps() >= 5){
          doRejectUpdate = true;
        }


        // Apply the update if valid
        if (!doRejectUpdate) {
            swerveDrive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        }
        
    }
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

  private boolean isRedAlliance() {
  return DriverStation.getAlliance().isPresent()
      && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }

  private Pose2d tagFace(){
    if (isRedAlliance()){
      return Constants.cordenadas.tag_10;
    } else {
      return Constants.cordenadas.tag_26;
    }
  }

  /*public double getTxDesiredDeg() {
  Translation2d objhub = swerveSubsystem.hubObj();

  double angleToHub = Math.toDegrees(Math.atan2(
    objhub.getY() - swerveDrive.getPose().getY(),
    objhub.getX() - swerveDrive.getPose().getX()
  ));

  double angleToFace = Math.toDegrees(Math.atan2(
    tagFace().getY() - swerveDrive.getPose().getY(),
    tagFace().getX() - swerveDrive.getPose().getX()
  ));

  double txDesired = angleToHub - angleToFace;
  return MathUtil.inputModulus(txDesired, -180.0, 180.0);
  } */

  public double limeTx () {
    return LimelightHelpers.getTX(limelightNames);
  }

  public double hubDis () {
    double[] pose = LimelightHelpers.getTargetPose_CameraSpace("limelight-derof");
    if (pose != null && pose.length >= 3 && LimelightHelpers.getTV("limelight-derof")) {
      return pose[2];
    } else {
      return 0.5;
    }
  }

}


