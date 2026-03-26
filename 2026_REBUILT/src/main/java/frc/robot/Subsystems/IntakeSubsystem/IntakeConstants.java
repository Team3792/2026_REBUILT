package frc.robot.Subsystems.IntakeSubsystem;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
    
    public final static double kIntakeVoltage = 10;
    public final static double kDrivePivotVoltage = 3;
    public final static double kPivotHoldsVoltage = 0.5;
    public final static double kPivotTheshold = 2;

    public final static double kIntakePosition = 12.3; // need to set
    public final static double kHoldPosition = 6.15;
    public final static double kStowedPosition = 0; //need to set

    public static final double kEncoderOffset = 0;
    //8 to 56
    //16 to 64
    //16 to 34

    public static final double kGearRatio = (8/56) * (16/64) * (16/34);


    public final static double kP = 2;
    public final static double kI = 0;
    public final static double kD = 0;
    public final static double kVelocityFF = 0;
    public final static double kFrictionFF = 0;
    public final static double kGravityFF = 0;
    
    //where an angle of 0 degrees is intake vertical stow position
    public static double GravityFF(double intakeAngle){
        return 9.81 * Math.sin(intakeAngle) * kGravityFF;
    }

    public static TalonFXConfiguration applyIntakeConfig(){
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;//TO-DO; make accurate
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 40;
        return config;
    }

    public static TalonFXConfiguration applyPivotConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();

        //Motor Configs
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //TO-DO; make accurate
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 30;

        //Slot 0 configs
        Slot0Configs slot0 = config.Slot0;
        slot0.kS = kFrictionFF;
        slot0.kV = kVelocityFF;
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;

        //Motion Magic Configs
        MotionMagicConfigs mm = config.MotionMagic;
        mm.MotionMagicCruiseVelocity = 30;//80;
        mm.MotionMagicAcceleration = 160;

        return config;
    }
}
