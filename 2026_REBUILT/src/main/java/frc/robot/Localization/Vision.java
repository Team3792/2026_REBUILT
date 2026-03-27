// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Localization;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  public PhotonCamera frontCam, backCam;

  public Vision() {

    frontCam = new PhotonCamera(VisionConstants.kFrontCamName);
    backCam = new PhotonCamera(VisionConstants.kBackCamName);

    //Can do HSV yellow detection to do fuel detection. 
    //Need to do tuning and setup in photonvision dashboard
    //create a new pipeline
    //fuelCam = new PhotonCamera("fuelCam");
  }

  public boolean hasTarget(PhotonCamera cam){
    var result = cam.getLatestResult();
    boolean hasTarget = result.hasTargets();
    return hasTarget;
  }

  public List<PhotonPipelineResult> getFrontCamPipelineResults(){
    List<PhotonPipelineResult> pipelineResults = frontCam.getAllUnreadResults();
    return pipelineResults;
  }
 
  public List<PhotonPipelineResult> getBackCamPipelineResults(){
    List<PhotonPipelineResult> pipelineResults = backCam.getAllUnreadResults();
    return pipelineResults;
  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("back cam", hasTarget(backCam));
    // SmartDashboard.putBoolean("front cam", hasTarget(frontCam));
  }
}
