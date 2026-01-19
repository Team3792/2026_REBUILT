package frc.robot.SwerveSubsystem;

public class SwerveConstants {
    
    public static final double kMaxSpeedMetersPerSecond = 5;

    public static final double kTrackWidth = 0.6;
    public static final double kWheelBase = 0.7;

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        // Front right
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        // Back left
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        // Back right
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );
}
