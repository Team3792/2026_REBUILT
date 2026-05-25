package frc.robot.Subsystems.ShooterSubsystem.Hood;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public class HoodConstants {

    public static final double kP = 0.2;
    public static final double kI = 0;
    public static final double kD = 0.005;
    public static final double kFrictionFF = 0;
    public static final double kMotorRatio = 25;
    public static final double kHoodRatio = 300.0/36;

    // All in degrees
    public static final double kMinAngle = 16;
    public static final double kMaxAngle = 50;
    public static final double kZeroAngle = 16;
    public static final double kHubHoodAngle = 20;
    public static final double kAngleTolerance = 0.5;

    public static SparkMaxConfig configHood() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(40);
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);
        config.closedLoop.pid(kP, kI, kD);
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.maxOutput(0.8);
        config.closedLoop.minOutput(-0.8);
        return config;
    }
}
