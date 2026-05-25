package frc.robot.Localization;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final String kFrontCamName = "front";
    public static final String kBackCamName = "back";
    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    //Robot pose from center of robot
    public static final Transform3d kRobotToFrontCam = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(2.75),
            Units.inchesToMeters(0),
            Units.inchesToMeters(20.625)),
        new Rotation3d(
            0,
            11 * Math.PI / 180,
            0)
    );

    public static final Transform3d kRobotToBackCam = new Transform3d(new Translation3d(Units.inchesToMeters(-11.5),
    Units.inchesToMeters(0.5),Units.inchesToMeters(21.25)), new Rotation3d(0,12 * Math.PI / 180, Math.PI));

    public static final Transform2d kRobotToShooter = new Transform2d(
        new Translation2d(
            Units.inchesToMeters(-5.25),
            Units.inchesToMeters(-0)),
        new Rotation2d(0)
    );
}

