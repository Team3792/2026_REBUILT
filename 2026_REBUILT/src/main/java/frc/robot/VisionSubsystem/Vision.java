// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.VisionSubsystem;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  
  PhotonCamera frontCam, backCam;

  public Vision() {

    PhotonCamera frontCam = new PhotonCamera(VisionConstants.kFrontCamName);
    PhotonCamera backCam = new PhotonCamera(VisionConstants.kBackCamName);
  }

  PhotonPoseEstimator frontCamPoseEstimation = new PhotonPoseEstimator(VisionConstants.fieldLayout, VisionConstants.kRobotToFrontCam);
  PhotonPoseEstimator backCamPoseEstimation = new PhotonPoseEstimator(VisionConstants.fieldLayout, VisionConstants.kRobotToBackCam);

  public boolean hasTarget(PhotonCamera camera) {
    
    var result = camera.getLatestResult();

    if (!result.hasTargets()){
      return false;
    } else {
      return true;
    } 
  }

  public int getTagID(PhotonCamera camera) {
    var result = camera.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    int tagID = target.getFiducialId();

    return tagID;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("VisionSubsystem/Vision", getTagID(frontCam));
    SmartDashboard.putBoolean("VisionSubsystem/Vision", hasTarget(frontCam));

  }
}
