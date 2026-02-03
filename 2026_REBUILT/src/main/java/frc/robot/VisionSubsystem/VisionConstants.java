// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.VisionSubsystem;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class VisionConstants {

    public static final String kFrontCamName = "frontCam";
    public static final String kBackCamNamei = "backCam";

    public static final Transform3d kRobotToFrontCam = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
    public static final Transform3d kRobotToBackCam = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
}
