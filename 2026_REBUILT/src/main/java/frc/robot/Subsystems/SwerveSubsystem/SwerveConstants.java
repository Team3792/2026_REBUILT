package frc.robot.Subsystems.SwerveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.util.ControlConfig.PIDConfig;
import frc.robot.util.ControlConfig.ProfiledPIDConfig;

public class SwerveConstants {
    
    public static final double kMaxSpeedMetersPerSecond = 5; //TODO: verfiy
    public static final double kDeadBand = 0.08;
    public static final double kMaxLinearVelocity = 4.0;
    public static final double kMaxOmega = 4.0;

    public static final double kTrackWidth = Units.inchesToMeters(22.5); //TODO: verify
    public static final double kWheelBase = Units.inchesToMeters(22.5); //TODO: verify


    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );

    //Used for path planner
    public static final PIDConfig kTranslationControllerConfig = new PIDConfig(1, 0.1, 0);
    public static final PIDConfig kTurnControllerConfig = new PIDConfig(2, 0.01, 0);
    
    //Used for internal alignment
    public static final ProfiledPIDConfig kHeadingLockControllerConfig = new ProfiledPIDConfig(2, 0.01, 0, 6, 12);
}
