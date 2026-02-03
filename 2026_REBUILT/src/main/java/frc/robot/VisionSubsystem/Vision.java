// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.VisionSubsystem;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  PhotonCamera shooterCam;
  public Vision() {
    shooterCam = new PhotonCamera(VisionConstants.kShooterCamName);
  }

  public boolean hasTarget() {
    var result = shooterCam.getLatestResult();
    boolean hasTarget;
    if (!result.hasTargets()){
      hasTarget = false;
    } else {
      hasTarget = true;
    }
    return hasTarget;
  }

  public int getTagID() {
    var result = shooterCam.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    int tagID = target.getFiducialId();

    return tagID;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("VisionSubsystem/Vision", getTagID());
    SmartDashboard.putBoolean("VisionSubsystem/Vision", hasTarget());

  }
}
