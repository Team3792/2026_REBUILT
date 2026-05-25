package frc.robot.Localization;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  public PhotonCamera frontCam, backCam;

  public Vision() {
    frontCam = new PhotonCamera(VisionConstants.kFrontCamName);
    backCam = new PhotonCamera(VisionConstants.kBackCamName);
  }

  public List<PhotonPipelineResult> getFrontCamPipelineResults(){
    List<PhotonPipelineResult> pipelineResults = frontCam.getAllUnreadResults();
    return pipelineResults;
  }
 
  public List<PhotonPipelineResult> getBackCamPipelineResults(){
    List<PhotonPipelineResult> pipelineResults = backCam.getAllUnreadResults();
    return pipelineResults;
  }
}
