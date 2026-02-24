package frc.robot.Subsystems.ClimbSubsytem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimbConstants {
    public final static double kClimbVoltage = 5.0;

    public static TalonFXConfiguration getConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TO-DO; make accurate
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        //Threshold in motor rotations, use gear ratio to calculate
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0; 
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60;

        return config;
    }
}
