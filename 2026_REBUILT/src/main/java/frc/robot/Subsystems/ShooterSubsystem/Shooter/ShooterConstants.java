package frc.robot.Subsystems.ShooterSubsystem.Shooter;


import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

public class ShooterConstants {

    //ball tracking (not utilized on 2026 robot)
    public static final double kLightSensorThreshold = 0.5;
    public static final double lightSensorDetectionRange = 0.01; //meters
    public static final double kballSeperationTime = 0.1;
    public static final double kFlywheelBallLoadCurrent = 0; //amps

    //pid & FF
    public static final double kV = 0.11764705882;
    public static final double kS = 0.23;
    public static final double kMMAccel = 200;
    public static final double kMMJerk = 2100;
    public static final double kP = 0.5; 
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFrictionFF = 0;

    public static TalonFXConfiguration applyRightMotorConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 70;

        Slot0Configs slot0 = config.Slot0;
        slot0.kS = kS;
        slot0.kV = kV;
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;

        MotionMagicConfigs mmconfig = config.MotionMagic;
        mmconfig.MotionMagicAcceleration = kMMAccel;
        mmconfig.MotionMagicJerk = kMMJerk;
        return config;
    }
    public static TalonFXConfiguration applyLeftMotorConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 70;

        Slot0Configs slot0 = config.Slot0;
        slot0.kS = kS;
        slot0.kV = kV;
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;

        MotionMagicConfigs mmconfig = config.MotionMagic;
        mmconfig.MotionMagicAcceleration = kMMAccel;
        mmconfig.MotionMagicJerk = kMMJerk;
        return config;
    }

    private static final double kFlywheelDiameterInches = 4; //Flywheel Diameter in Inches
    public static final double kFlywheelDiameter =  Units.inchesToMeters(kFlywheelDiameterInches); //Flywheel Diameter in Meters
    public static final double kFlywheelRadius = kFlywheelDiameter / 2; //Flywheel Radius in Meters

    public static final double kRPSIncrement = 1;
    public static final double maxShooterRPS = 6000;
    public static final double kHubRPS = 45;
    public static final double kRPSTolerance = 3;

    //For calibrated shots, see separate ShotCalculator class
}
