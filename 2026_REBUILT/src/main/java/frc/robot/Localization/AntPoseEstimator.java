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
    //List<PhotonPipelineResult> frontCamPipelineResults = vision.getFrontCamPipelineResults();

    public AntPoseEstimator (SwerveDriveKinematics kin, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions,
        Pose2d initialPoseMeters){

        super(kin, gyroAngle, modulePositions, initialPoseMeters);
    }

    private boolean isVisionDataValid(EstimatedRobotPose pose){
        double distance = pose.estimatedPose.toPose2d().getTranslation().getDistance(null); //TODO: add robot pose here (but it lives in swerve so awk))
        if (distance > 6.0) { //TODO: find good distance threshold
            return false;
        }
        return true;
    }

    //TODO: implement scaling
    public Matrix<N3, N1> getVisionStd(Optional<EstimatedRobotPose> robotPose, List<PhotonTrackedTarget> targets){

        Matrix<N3, N1> std = VecBuilder.fill(0.0, 0.0, 0.0); 

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

        if (numTags == 0){
            std = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            avgDis /= numTags;
            double scaleFactor = Math.pow(avgDis, 2) / Math.sqrt(numTags); //Stole from random team

            std = VecBuilder.fill(0.2, 0.2, 0.7).times( 1 + (avgDis * avgDis /30)); //Stole from photonvision docs
        }

        return std;
    }

    private void updateVisionMeasurements(List<PhotonPipelineResult> pipelineResults) {
        Optional<EstimatedRobotPose> visionEstFront = Optional.empty();

        for (var result : pipelineResults){
            visionEstFront = frontCamPoseEstimator.estimateCoprocMultiTagPose(result);
            List<PhotonTrackedTarget> targets = result.getTargets();
            if (visionEstFront.isEmpty()){
                continue;
            }
            if (!isVisionDataValid(visionEstFront.get())){
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

    public void updateAllVisionMeasurements(){
        updateVisionMeasurements(vision.getFrontCamPipelineResults());
        updateVisionMeasurements(vision.getBackCamPipelineResults());
    }

    //TODO: Find a way to update odometry with different stds
    public void updateOdometry(Rotation2d gyroYaw, ModulePosition[] modPositions){
    }
}
