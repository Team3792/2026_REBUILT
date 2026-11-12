package frc.robot.Subsystems.IntakeSubsystem;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
    
    public final static double kIntakeVoltage = 5;//6;
    public final static double kDrivePivotVoltage = 3;
    public final static double kPivotTheshold = 2;

    public final static double kIntakePosition = 22.50;
    public final static double kFramePerimStow = 5.41;
    public final static double kStowOut = 14.268;
    public final static double kZeroPosition = 0;

    public static final double kGearRatio = (8./56) * (16./64) * (16./34); 
    public final static double kP = 2;
    public final static double kI = 0.1;
    public final static double kD = 0;
    public final static double kVelocityFF = 0;
    public final static double kFrictionFF = 0;
    public final static double kGravityFF = 0;

    public static TalonFXConfiguration applyLeftIntakeConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 50;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Voltage.PeakForwardVoltage = 16;

        return config;
    }

    public static TalonFXConfiguration applyRightIntakeConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 50;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Voltage.PeakForwardVoltage = 16;

        return config;
    }

    public static TalonFXConfiguration applyPivotConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();

        //Motor Configs
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 30;

        //Slot 0 configs
        Slot0Configs slot0 = config.Slot0;
        slot0.kS = kFrictionFF;
        slot0.kV = kVelocityFF;
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;

        //Motion Magic Trapezoidal Profile Configs
        MotionMagicConfigs mm = config.MotionMagic;
        mm.MotionMagicCruiseVelocity = 30;
        mm.MotionMagicAcceleration = 160;

        return config;
    }
}
