// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Localization;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class VisionConstants {

    public static final String kFrontCamName = "front";
    public static final String kBackCamName = "back";


    //Robot pose always from center of robot
    public static final Transform3d kRobotToFrontCam = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-6.25),
            Units.inchesToMeters(-0.625),
            Units.inchesToMeters(20.875)),
        new Rotation3d(
            0,
            12 * Math.PI / 180,
            0)
    );

        
    public static final Transform3d kRobotToBackCam = new Transform3d(new Translation3d(-0.625,-8,20.875), new Rotation3d(0,12 * Math.PI / 180, Math.PI));
    // public static final Transform3d kRobotToShooter = new Transform3d(
    //     new Translation3d(
    //         Units.inchesToMeters(8),
    //         Units.inchesToMeters(-8.5),
    //         Units.inchesToMeters(18)),
    //     new Rotation3d(
    //         0,
    //         0,
    //         0)
    // );

    public static final Pose2d kRobotToShooter = new Pose2d(
        new Translation2d(
            Units.inchesToMeters(-8.5),
            Units.inchesToMeters(8)),
        new Rotation2d(0)
    );

    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
}

