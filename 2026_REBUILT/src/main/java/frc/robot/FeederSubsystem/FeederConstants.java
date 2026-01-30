package frc.robot.FeederSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FeederConstants {
    public final static double kFeedVoltage = 5;

    public static TalonFXConfiguration getConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;//TO-DO; make accurate
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        return config;
    }
}
