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

import edu.wpi.first.math.Matrix;
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
public class AntPose extends SwerveDrivePoseEstimator {

    Vision vision = new Vision();
    PhotonPoseEstimator frontCamPoseEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout, VisionConstants.kRobotToFrontCam);
    PhotonPoseEstimator backCamPoseEstimator = new PhotonPoseEstimator(VisionConstants.kFieldLayout, VisionConstants.kRobotToBackCam);
    List<PhotonPipelineResult> frontCamPipelineResults = vision.getFrontCamPipelineResults();
    

    public AntPose (SwerveDriveKinematics kin, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions,
        Pose2d initialPoseMeters){

        super(kin, gyroAngle, modulePositions, initialPoseMeters);
    }



    //validity based on distance tag is from camera, and ambiguity of detected tag.
    //further distance means less ambiguity needed for pose to be tossed out
    //TODO: find a way to hold timestamps and store them and access them
    public List<EstimatedRobotPose> getFrontCamAcceptedPoses(List<EstimatedRobotPose> poses) {
        // check if the data is valid
        // Can check abiguity of tag, distance to tag, etc.
        List<EstimatedRobotPose> acceptedFrontPoses = new ArrayList<EstimatedRobotPose>();
        for (int i = 0; i < poses.size(); i++){
            var pose = poses.get(i);
            /** 
             * TODO: change it from purely disstance to having it compare to our max velcoity
             * compare calculated velocity to max velocity and reject if it is over
             * bc based on diff times
             * maybe useless tho
             * */ 
            double distance = pose.estimatedPose.toPose2d().getTranslation().getDistance(null); //TODO: add robot pose here (but it lives in swerve so awk))
            if (distance > 6.0) { //TODO: find good distance threshold
                continue;
            }
            acceptedFrontPoses.add(pose);
        }
        return acceptedFrontPoses;
    }

    public List<PhotonTrackedTarget> getFrontCamTargets(List<PhotonPipelineResult> results){
        List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
        for (var result : results) {
            List<PhotonTrackedTarget> resultTargets = result.getTargets();
            for (var target: resultTargets){
                targets.add(target);
            }
        }
    return targets;
  }

    public List<EstimatedRobotPose> getBackCamAcceptedPoses(List<EstimatedRobotPose> poses) {
        List<EstimatedRobotPose> acceptedBackPoses = new ArrayList<EstimatedRobotPose>();
        for (int i = 0; i < poses.size(); i++){
            var pose = poses.get(i);
            double distance = pose.estimatedPose.toPose2d().getTranslation().getDistance(null); //TODO: add robot pose here (but it lives in swerve so awk))
            if (distance > 6.0) { //TODO: find good distance threshold
                continue;
            }
            acceptedBackPoses.add(pose);
        }
        return acceptedBackPoses;
    }




    public Matrix<N3, N1> getVisionStd(Optional<EstimatedRobotPose> robotPose, List<PhotonTrackedTarget> targets){
    }


    public List<EstimatedRobotPose> getFrontCamPoses(List<PhotonPipelineResult> results){
        Optional<EstimatedRobotPose> visionEstimate = Optional.empty();
        List<EstimatedRobotPose> visionEstimates = new ArrayList<EstimatedRobotPose>();

        for (var result : results){
            visionEstimate = frontCamPoseEstimator.estimateCoprocMultiTagPose(result);
            if (visionEstimate.isEmpty()){
                continue;
            }
            visionEstimates.add(visionEstimate.get());
        }
        return visionEstimates;
    }

    // TODO: add way to make pose2d 
    public void something(List<PhotonPipelineResult> pipelineResults) {
        Optional<EstimatedRobotPose> visionEstFront = Optional.empty();

        for (var result : pipelineResults){
            visionEstFront = frontCamPoseEstimator.estimateCoprocMultiTagPose(result);
            List<PhotonTrackedTarget> targets = result.getTargets();
            if (visionEstFront.isEmpty()){
                continue;
            }
            Matrix<N3,N1> std = getVisionStd(visionEstFront, targets);
            addVisMeasurement(visionEstFront, result.getTimestampSeconds(), std);
        }
    }

    public void addVisMeasurement(Optional<EstimatedRobotPose> visionPose, double timestamp, Matrix<N3,N1> visionSTD){
        if (visionPose.isEmpty()){
            return;
        }
        this.addVisionMeasurement(visionPose.get().estimatedPose.toPose2d(), timestamp, visionSTD);
    }


    public void getVisionData(){}

}
