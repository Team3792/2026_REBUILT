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
    public static final double kTurnRatio = 15.4299;
    public static final double kMetersPerRotation = Math.PI/7.36*Units.inchesToMeters(4);
    public static final double kRadiansPerRotation = Math.PI*2 / kTurnRatio;
    

    //Module Configurations.     Offsets need to be tuned
    public static final ModuleConfig kFrontLeft = new ModuleConfig(0, 1, 2, 0.0);
    public static final ModuleConfig kFrontRight = new ModuleConfig(3, 4, 5, 0.0);
    public static final ModuleConfig kBackLeft = new ModuleConfig(6, 7, 8, 0.0);
    public static final ModuleConfig kBackRight = new ModuleConfig(9, 10, 11, 0.0);



    public static record ModuleConfig (int driveAddress, int turnAddress, int encoderAddress, double encoderOffset){}

    public static MagnetSensorConfigs getEncoderConfig(double offset){
        MagnetSensorConfigs config = new MagnetSensorConfigs();
        config.MagnetOffset = offset;
        config.SensorDirection = SensorDirectionValue.Clockwise_Positive; //can be changed if needed
        config.AbsoluteSensorDiscontinuityPoint = 1.0;
        return config;
    }

    public static TalonFXConfiguration getDriveConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        Slot0Configs slot0 = config.Slot0;
        slot0.kS = 0.17;
        slot0.kV = 0.11;
        slot0.kP = 0.001;
        slot0.kI = 0;
        slot0.kD = 0;
        
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //can be changed if needed
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        return config;
    }

    public static TalonFXConfiguration getTurnConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.CurrentLimits.StatorCurrentLimit = 30;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        return config;


    }



}
