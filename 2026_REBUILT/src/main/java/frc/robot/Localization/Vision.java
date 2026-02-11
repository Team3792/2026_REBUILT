// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Localization;

import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  
  //PhotonCamera frontCam, backCam;
  public static PhotonCamera frontCam, backCam;
  PhotonPoseEstimator frontCamPoseEstimation;
  PhotonPoseEstimator backCamPoseEstimation;

  public Vision() {

    frontCam = new PhotonCamera(VisionConstants.kFrontCamName);
    backCam = new PhotonCamera(VisionConstants.kBackCamName);

    frontCamPoseEstimation = new PhotonPoseEstimator(VisionConstants.kFieldLayout, VisionConstants.kRobotToFrontCam);
    backCamPoseEstimation = new PhotonPoseEstimator(VisionConstants.kFieldLayout, VisionConstants.kRobotToBackCam);
  }

  public boolean hasTarget(PhotonCamera cam){
    var result = cam.getLatestResult();
    boolean hasTarget = result.hasTargets();
    return hasTarget;
  }

  public Optional<EstimatedRobotPose> getFrontCamPoseLatest(){
    var result = frontCam.getLatestResult();  
    return frontCamPoseEstimation.estimateCoprocMultiTagPose(result);
  }

  public Optional<EstimatedRobotPose> getBackCamPoseLatest(){
    var result = backCam.getLatestResult();
    return backCamPoseEstimation.estimateCoprocMultiTagPose(result);
  }

  public List<EstimatedRobotPose> getFrontCamResults(){
    Optional<EstimatedRobotPose> visionEstimate = Optional.empty();
    List<EstimatedRobotPose> visionEstimates = new ArrayList<EstimatedRobotPose>();
    for (var result : frontCam.getAllUnreadResults()) {
      visionEstimate = frontCamPoseEstimation.estimateCoprocMultiTagPose(result);
      if (visionEstimate.isEmpty()) {
        continue;
      }
      visionEstimates.add(visionEstimate.get());
    }
    return visionEstimates;
  }

  public List<EstimatedRobotPose> getBackCamResults(){
    Optional<EstimatedRobotPose> visionEstimate = Optional.empty();
    List<EstimatedRobotPose> visionEstimates = new ArrayList<EstimatedRobotPose>();
    for (var result : frontCam.getAllUnreadResults()) {
      visionEstimate = frontCamPoseEstimation.estimateCoprocMultiTagPose(result);
      if (visionEstimate.isEmpty()) {
        continue;
      }
      visionEstimates.add(visionEstimate.get());
    }
    return visionEstimates;
  }
  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("VisionSubsystem/frontCam", hasTarget(frontCam));
    SmartDashboard.putBoolean("VisionSubsystem/backCam", hasTarget(backCam));   

    getFrontCamPose();
    getBackCamPose();
  }
}
