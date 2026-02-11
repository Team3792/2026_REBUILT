// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Localization;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
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
   
  public int getTagID(PhotonCamera cam) {
    var result = cam.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      int tagID = target.getFiducialId();
      return tagID;
    } else {
      return -1; // Return -1 if no targets are detected
    }
  }

  public double getTagYaw(PhotonCamera cam) {
    double tagYaw = 0;
    var result = cam.getLatestResult();
    if (result.hasTargets()){
      var target = result.getBestTarget();
      tagYaw = target.getYaw();
    } 
    return tagYaw;
  }

  //Return Pose2d of tag
  //Still need implementation
  public Pose2d getTagPose(PhotonCamera cam) {
    return new Pose2d();
  }

  //robot pose estimation based on combination of all tags seen by camera
  public Optional<EstimatedRobotPose> getRobotPose(PhotonCamera cam){
    var result = cam.getLatestResult();
    return frontCamPoseEstimation.estimateCoprocMultiTagPose(result);
  }

  public  Optional<EstimatedRobotPose> frontCamPose(){
    return getRobotPose(frontCam);
  }
  public  Optional<EstimatedRobotPose> backCamPose(){
    return getRobotPose(backCam);
  }
   


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("VisionSubsystem/frontCam", hasTarget(frontCam));
    SmartDashboard.putBoolean("VisionSubsystem/backCam", hasTarget(backCam));
    SmartDashboard.putNumber("VisionSubsystem/frontCamTagID", getTagID(frontCam));
    SmartDashboard.putNumber("VisionSubsystem/backCamTagID", getTagID(backCam));
    SmartDashboard.putNumber("VisionSubsystem/frontCamTagYaw", getTagYaw(frontCam));
    SmartDashboard.putNumber("VisionSubsystem/backCamTagYaw", getTagYaw(backCam));
    
  }
}
