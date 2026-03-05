package frc.robot.Subsystems.SwerveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {
    
    public static final double kMaxSpeedMetersPerSecond = 5;

    public static final double kTrackWidth = Units.inchesToMeters(22.5);
    public static final double kWheelBase = Units.inchesToMeters(22.5);


    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );

    public static double kDeadBandValue = 0.08;
    public static double kMaxLinearVelocity = 4.0;
    public static double kMaxOmega = 4.0;


    public static PIDController kTranslationController = new PIDController(1, 0.1, 0);
    public static ProfiledPIDController kRotationController = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(3.14, 6.28));
    
    private double kDeadBandTreshold = 0.1;

    public double deadBand(double input){
        if(input > kDeadBandTreshold){
            return input;
        }else{
            return 0;
        }
    }
}
