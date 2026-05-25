package frc.robot.Localization;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AntPoseEstimator extends SwerveDrivePoseEstimator {
    Vision vision = new Vision();
    PhotonPoseEstimator frontCamPoseEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout, VisionConstants.kRobotToFrontCam);
    PhotonPoseEstimator backCamPoseEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout, VisionConstants.kRobotToBackCam);
    Field2d field = new Field2d();

    public AntPoseEstimator(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d initialPoseMeters){
        super(kinematics, gyroAngle, modulePositions, initialPoseMeters);
        SmartDashboard.putData("AntPose Pose Estimate", field);
    }

    private boolean isVisionDataValid(EstimatedRobotPose pose){
        double distance = pose.estimatedPose.toPose2d().getTranslation().getDistance(this.getEstimatedPosition().getTranslation());
        if (distance > 6.0) 
            return false;
        
        return true;
    }

    //Returns standard deviations for kalman filter (determines how much to trust an estimated vision pose)
    private Matrix<N3, N1> getVisionStd(Optional<EstimatedRobotPose> robotPose, List<PhotonTrackedTarget> targets){
        Matrix<N3, N1> std = VecBuilder.fill(0.0, 0.0, 0.0); 
        double baseXY = 0.2;
        double baseT = 0.3;
        int numTags = 0;
        double avgDis = 0.0;        

        for (var target : targets){
            var targetPose = frontCamPoseEstimator.getFieldTags().getTagPose(target.getFiducialId());
            if (targetPose.isEmpty()){
                continue;
            }
            numTags++;
            avgDis += targetPose.get().toPose2d().getTranslation().getDistance(this.getEstimatedPosition().getTranslation());
        }
        avgDis /= numTags;

        if (numTags == 0){
            std = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } 
        else if (numTags == 1) { 
            std = VecBuilder.fill(1.0, 1.0, 2.0);
        }
        else{
            double disScale = avgDis / 3.0; //3 meters is baseline for good reading
            double tagScale = 1.0 / Math.sqrt(numTags);
            double stdXY = baseXY * disScale * tagScale;
            double stdT = baseT * tagScale;
            std = VecBuilder.fill(stdXY, stdXY, stdT);
        }
        return std;
    }

    private void updateVisionMeasurementsFront(List<PhotonPipelineResult> pipelineResults) {
        Optional<EstimatedRobotPose> visionEst;
            for (var result : pipelineResults){
                List<PhotonTrackedTarget> targets = result.getTargets();
                visionEst = frontCamPoseEstimator.estimateCoprocMultiTagPose(result);

                if (visionEst.isEmpty()){
                    continue;
                }
                if (!isVisionDataValid(visionEst.get())){
                    continue;
                }

                Matrix<N3,N1> std = getVisionStd(visionEst, targets);
                addVisionMeasurement(visionEst.get().estimatedPose.toPose2d(), result.getTimestampSeconds(), std);
            }
    }

    private void updateVisionMeasurementsBack(List<PhotonPipelineResult> pipelineResults) {
        Optional<EstimatedRobotPose> visionEst;
            for (var result : pipelineResults){
                List<PhotonTrackedTarget> targets = result.getTargets();
                visionEst = backCamPoseEstimator.estimateCoprocMultiTagPose(result);

                if (visionEst.isEmpty()){
                    continue;
                }
                if (!isVisionDataValid(visionEst.get())){
                    continue;
                }

                Matrix<N3,N1> std = getVisionStd(visionEst, targets);
                addVisionMeasurement(visionEst.get().estimatedPose.toPose2d(), result.getTimestampSeconds(), std);
            }
    }

    public void updateAllVisionMeasurements(){
        updateVisionMeasurementsFront(vision.getFrontCamPipelineResults());
        updateVisionMeasurementsBack(vision.getBackCamPipelineResults());
    }

    public void updateAll(Rotation2d gyroHeading, SwerveModulePosition[] modulePositions){
        super.update(gyroHeading, modulePositions);  //odometry update
        updateAllVisionMeasurements();
        field.setRobotPose(getEstimatedPosition());
    }
}
