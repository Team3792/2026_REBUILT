package frc.robot.IntakeSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
    
    public final static double intakeVoltage = 5;
    public final static int kIntakeCanID = 0; // need to set
    public final static int kPivotCanID = 0; // need to set
    public final static int kEncoderCanID = 0; // need to set

    public final static double kIntakePosition = 0; // need to set
    public final static double kStowedPosition = 0; //need to set

    public final static double kP = 0;
    public final static double kI = 0;
    public final static double kD = 0;

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
