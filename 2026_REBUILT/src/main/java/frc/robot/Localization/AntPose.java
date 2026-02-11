// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Localization;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

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
    

    public AntPose (SwerveDriveKinematics kin, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions,
        Pose2d initialPoseMeters){

        super(kin, gyroAngle, modulePositions, initialPoseMeters);
    }

    //validity based on distance tag is from camera, and ambiguity of detected tag.
    //further distance means less ambiguity needed for pose to be tossed out
    //TODO: find a way to hold timestamps and store them and access them
    public List<EstimatedRobotPose> getFrontCamAcceptedPoses() {
        // check if the data is valid
        // Can check abiguity of tag, distance to tag, etc.
        List<EstimatedRobotPose> poses = vision.getFrontCamResults();
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

    public List<EstimatedRobotPose> getBackCamAcceptedPoses() {
        List<EstimatedRobotPose> backPoses = vision.getBackCamResults();
        List<EstimatedRobotPose> acceptedBackPoses = new ArrayList<EstimatedRobotPose>();
        for (int i = 0; i < backPoses.size(); i++){
            var pose = backPoses.get(i);
            double distance = pose.estimatedPose.toPose2d().getTranslation().getDistance(null); //TODO: add robot pose here (but it lives in swerve so awk))
            if (distance > 6.0) { //TODO: find good distance threshold
                continue;
            }
            acceptedBackPoses.add(pose);
        }
        return acceptedBackPoses;
    }

    // IDK if we will ever use this
    public List<EstimatedRobotPose> getAcceptedPoses() {
        List<EstimatedRobotPose> frontPoses = getFrontCamAcceptedPoses();
        List<EstimatedRobotPose> backPoses = getBackCamAcceptedPoses();
        List<EstimatedRobotPose> allPoses = new ArrayList<>();
        allPoses.addAll(frontPoses);
        allPoses.addAll(backPoses);
        return allPoses;
    }

    public Matrix<N3, N1> getVisionStd(Optional<EstimatedRobotPose> robotPose){ 

    }



    public void getVisionData(){}
}
