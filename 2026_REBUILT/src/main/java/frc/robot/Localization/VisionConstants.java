// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Localization;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class VisionConstants {

    public static final String kFrontCamName = "frontCam";
    public static final String kBackCamName = "backCam";

    public static final Transform3d kRobotToFrontCam = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
    public static final Transform3d kRobotToBackCam = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));

    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
}

