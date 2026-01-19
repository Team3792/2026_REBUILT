package frc.robot.ClimbSubsytem;

public class ClimbConstants {

    public final static double kLeftClimbID = 0; //TO-DO; set ID
    public final static double kRightClimbID = 0; //TO-DO; set ID

    public final static double kClimbVoltage = 5.0;

    public TalonFXConfiguration getConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TO-DO; make accurate
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        return config;
    }
}
