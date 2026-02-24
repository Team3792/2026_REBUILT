package frc.robot.Subsystems.IntakeSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
    
    public final static double kIntakeVoltage = 5;

    public final static double kIntakePosition = 0; // need to set
    public final static double kStowedPosition = 0; //need to set

    public final static double kP = 0;
    public final static double kI = 0;
    public final static double kD = 0;
    public final static double kFrictionFF = 0;
    public final static double kGravityFF = 0;
    
    //where an angle of 0 degrees is intake vertical stow position
    public static double GravityFF(double intakeAngle){
        return 9.81 * Math.sin(intakeAngle) * kGravityFF;
    }

    public static TalonFXConfiguration applyIntakeConfig(){
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;//TO-DO; make accurate
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        return config;
    }

    public static TalonFXConfiguration applyPivotConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TO-DO; make accurate
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        return config;
    }

    public static final double kEncoderOffset = 0;
}
