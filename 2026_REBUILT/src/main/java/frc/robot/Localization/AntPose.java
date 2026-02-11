// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Localization;

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

    public AntPose (SwerveDriveKinematics kin, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions,
        Pose2d initialPoseMeters, Matrix<N3, N1> stateStdDevs,Matrix<N3, N1> visionStdDevs) {
        super(kin, gyroAngle, modulePositions, initialPoseMeters, stateStdDevs, visionStdDevs);
    }

    public boolean isDataValid() {
        // check if the data is valid
        // Can check abiguity of tag, distance to tag, etc.
        return true; // Placeholder
    }
}
