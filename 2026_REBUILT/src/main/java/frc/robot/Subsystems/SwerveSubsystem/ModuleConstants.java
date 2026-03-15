package frc.robot.Subsystems.SwerveSubsystem;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;

public class ModuleConstants {

    //Conversion Factors
    public static final double kTurnRatio = 25.9;
    public static final double kDriveRatio = 6.0; //1/6 wheel rotations per motor rotation
    public static final double kMetersPerWheelRotation = Math.PI * Units.inchesToMeters(3.5); //3.5 is wheel diameter 
    public static final double kRadiansPerMotorRotation = Math.PI*2 / kTurnRatio;
    public static final double kMetersPerMotorRotation = kMetersPerWheelRotation / kDriveRatio;
    

    //Module Configurations.     Offsets need to be tuned
    public static final ModuleConfig kFrontLeft = new ModuleConfig(0, 1, 2, -0.618);
    public static final ModuleConfig kFrontRight = new ModuleConfig(3, 4, 5, -0.40);
    public static final ModuleConfig kBackLeft = new ModuleConfig(6, 7, 8, -0.6645);
    public static final ModuleConfig kBackRight = new ModuleConfig(9, 10, 11, -0.4445);



    public static record ModuleConfig (int driveAddress, int turnAddress, int encoderAddress, double encoderOffset){}

    public static MagnetSensorConfigs getEncoderConfig(double offset){
        MagnetSensorConfigs config = new MagnetSensorConfigs();
        config.MagnetOffset = offset;
        config.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; 
        config.AbsoluteSensorDiscontinuityPoint = 1.0;
        return config;
    }

    public static TalonFXConfiguration getDriveConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        Slot0Configs slot0 = config.Slot0;
        slot0.kS = 0.17;
        slot0.kV = 0.116;
        slot0.kP = 0;
        slot0.kI = 0;
        slot0.kD = 0;
        
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        return config;
    }

    public static TalonFXConfiguration getTurnConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.StatorCurrentLimit = 30;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        return config;
    }
}
