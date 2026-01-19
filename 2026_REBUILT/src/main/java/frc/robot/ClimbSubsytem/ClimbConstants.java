package frc.robot.ClimbSubsytem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimbConstants {

    public final static int kLeftClimbID = 0; //TO-DO; set ID
    public final static int kRightClimbID = 0; //TO-DO; set ID

    public final static double kClimbVoltage = 5.0;

    public static TalonFXConfiguration getConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TO-DO; make accurate
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        return config;
    }
}
