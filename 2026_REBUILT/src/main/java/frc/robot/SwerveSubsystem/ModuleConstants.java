package frc.robot.SwerveSubsystem;

public class ModuleConstants {
    
    //Conversion Factors
    public static final double kTurnRatio = 15.4299;
    public static final double kMetersPerRotation = Math.PI/7.36*Units.inchesToMeters(4);
    public static final double kRadiansPerRotation = Math.PI*2 / kTurnRatio;
    

    //Module Configurations.     Offsets need to be tuned
    public static final ModuleConfig kFrontLeft = new ModuleConfig(new CANAddress(1), new CANAddress(2), new CANAddress(3), 0.0);
    public static final ModuleConfig kFrontRight = new ModuleConfig(new CANAddress(4), new CANAddress(5), new CANAddress(6), 0.0);
    public static final ModuleConfig kBackLeft = new ModuleConfig(new CANAddress(7), new CANAddress(8), new CANAddress(9), 0.0);
    public static final ModuleConfig kBackRight = new ModuleConfig(new CANAddress(10), new CANAddress(11), new CANAddress(12), 0.0);

    public static record ModuleConfig (CANAddress driveAddress, CANAddress turnAddress, CANAddress encoderAddress, double encoderOffset){}

    public static MagnetSensorConfig getEncoderConfig(double offset){
        MagnetSensorConfigs config = new MagnetSensorConfigs();
        config.MagnetOffset = offset;
        config.SensorDirection = SensorDirectionValue.Clockwise_Positive; //can be changed if needed
        config.SensorAbsoluteSensorDiscontinuityPoint = 1.0
        return config;
    }

    public static TalonFXConfiguration getDriveConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();

        Slot0Configuration slot0 = config.Slot0;
        



        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //can be changed if needed
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        return config;
    }



}
