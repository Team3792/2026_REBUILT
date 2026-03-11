// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Localization;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModulePosition;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;


/** Add your docs here. */
public class AntPoseEstimator extends SwerveDrivePoseEstimator {

    Vision vision = new Vision();
    PhotonPoseEstimator frontCamPoseEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout, VisionConstants.kRobotToFrontCam);
    PhotonPoseEstimator backCamPoseEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout, VisionConstants.kRobotToBackCam);

    public AntPoseEstimator (SwerveDriveKinematics kin, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions,
        Pose2d initialPoseMeters){
        super(kin, gyroAngle, modulePositions, initialPoseMeters);
    }

    private boolean isVisionDataValid(EstimatedRobotPose pose){
        double distance = pose.estimatedPose.toPose2d().getTranslation().getDistance(this.getEstimatedPosition().getTranslation());
        if (distance > 6.0) { //TODO: find good distance threshold
            return false;
        }
        return true;
    }

    private Matrix<N3, N1> getVisionStd(Optional<EstimatedRobotPose> robotPose, List<PhotonTrackedTarget> targets){

        Matrix<N3, N1> std = VecBuilder.fill(0.0, 0.0, 0.0); 

        double baseXY = 0.2;
        double baseT = 0.3;

        int numTags = 0;

        // can change this to be max or to be weighted with combo of avg & max
        double avgDis = 0.0;
        double avgYaw = 0.0; 
        
        for (var target : targets){
            var targetPose = frontCamPoseEstimator.getFieldTags().getTagPose(target.getFiducialId());
            if (targetPose.isEmpty()){
                continue;
            }
            numTags++;
            avgDis += targetPose.get().toPose2d().getTranslation().getDistance(this.getEstimatedPosition().getTranslation());
            avgYaw += target.getYaw();
        }
        avgDis /= numTags;
        avgYaw /= numTags;

        if (numTags == 0){
            std = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else if (numTags == 1 && avgDis > 4.0){ // from photonvision docs. prob not needed
            std = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else if (numTags == 1) { 
            std = VecBuilder.fill(1.0, 1.0, 2.0); // need to tune
        }
        else{
            double disScale = avgDis / 3.0; //3 meters. can tune to be different
            double yawScale = avgYaw / 30.0; //30 degreees. can tune to be different
            /** Tune 0.05 value (refer as k)
             * tune by seeing what a good threshold for abg is
             * if k is a average threshold that can give good readings then use that
             * below k is a great reading so scale more
             * above k is a bad reading (0.2 is thershold for ambiguous so keep in mind)
             */
            double tagScale = 1.0 / Math.sqrt(numTags); //got from team 564, saw vid of local. and it was good

            double stdXY = baseXY * disScale * tagScale;
            double stdT = baseT * yawScale * tagScale;
            std = VecBuilder.fill(stdXY, stdXY, stdT);
        }
        return std;
    }

    private void updateVisionMeasurements(List<PhotonPipelineResult> pipelineResults, String camera) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        if (camera == "front"){
            for (var result : pipelineResults){
                visionEst = frontCamPoseEstimator.estimateCoprocMultiTagPose(result);
                List<PhotonTrackedTarget> targets = result.getTargets();
                if (visionEst.isEmpty()){
                    continue;
                }
                if (!isVisionDataValid(visionEst.get())){
                    continue;
                }
                Matrix<N3,N1> std = getVisionStd(visionEst, targets);
                addVisMeasurement(visionEst, result.getTimestampSeconds(), std);
            }
        } else{
            for (var result : pipelineResults){
                visionEst = backCamPoseEstimator.estimateCoprocMultiTagPose(result);
                List<PhotonTrackedTarget> targets = result.getTargets();
                if (visionEst.isEmpty()){
                    continue;
                }
                if (!isVisionDataValid(visionEst.get())){
                    continue;
                }
                Matrix<N3, N1> std = getVisionStd(visionEst, targets);
                addVisMeasurement(visionEst, result.getTimestampSeconds(), std);
            }
        }
    }

    private void addVisMeasurement(Optional<EstimatedRobotPose> visionPose, double timestamp, Matrix<N3,N1> visionSTD){
        if (visionPose.isEmpty()){
            return;
        }
        this.addVisionMeasurement(visionPose.get().estimatedPose.toPose2d(), timestamp, visionSTD);
    }

    public void updateAllVisionMeasurements(){
        updateVisionMeasurements(vision.getFrontCamPipelineResults(), "front");
        updateVisionMeasurements(vision.getBackCamPipelineResults(), "back");
    }

    //TODO: Find a way to update odometry with different stds
    //Lowkirkenuinly there may not be a way to do ts
    public void updateOdometry(Rotation2d gyroYaw, ModulePosition[] modPositions){}
}
