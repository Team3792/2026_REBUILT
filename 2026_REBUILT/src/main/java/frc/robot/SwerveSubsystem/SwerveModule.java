package frc.robot.SwerveSubsystem;

public class SwerveModule extends SubsystemBase{
    TalonFX drive, turn;
    CANcoder encoder;
    PIDController turnController;
    VelocityVoltage driveController;
    SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(0));

    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorID, int turnMotorID, int encoderID, double absoluteEncoderOffset) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        
        drive = new TalonFX(driveMotorID);
        turn = new TalonFX(turnMotorID);
        encoder = new CANcoder(encoderID);

        drive.setNeutralMode(NeutralModeValue.Coast);
        turn.setNeutralMode(NeutralModeValue.Coast);
      


        turnController = new PIDController(4, 0, 0.05);
        turnController.enableContinuousInput(-Math.PI, Math.PI);   

        resetEncoders();
    }

    public double getAbsoluteEncoderRad() {
        double angle = (encoder.getAbsolutePosition().getValueAsDouble()) * 2.0 * Math.PI - absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public double getDrivePostition(){
        return drive.getPosition().getValueAsDouble() * ModuleConstants.kMetersPerRotation;
    }

    public double getTurnPosition(){
        return encoder.getPosition().getValueAsDouble() * ModuleConstants.kTurnEncoderRadians2Rot;
    }

    public double getDriveVelocity(){
        return drive.getVelocity().getValueAsDouble() * ModuleConstants.kMetersPerRotation;
    }

    public double getTurnVelocity(){
        return encoder.getVelocity().getValueAsDouble() * ModuleConstants.kTurnEncoderRPM2RadPerSec;
    }
    
    public void resetEncoders(){
        drive.setPosition(0);
        //encoder.setPosition(getAbsoluteEncoderRad());
    }

    public Rotation2d getStateAngle(){
        return new Rotation2d(getTurnPosition());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), getStateAngle());
    }

    public void desiredState(SwerveModuleState state){
        // if (Math.abs(state.speedMetersPerSecond) < 0.001){
        //     stop();
        //     return;
        // }
        //state = SwerveModuleState.optimize(state, getStateAngle());
        //drive.set(state.speedMetersPerSecond / ModuleConstants.kMaxSpeedMetersPerSecond);
        turn.setVoltage(turnController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
    }

    public void setState(){
        
    }

    public void incrementTurn(SwerveModuleState state){
        double targetAngle = getAbsoluteEncoderRad() + (Math.PI * 0.5);
        turn.setVoltage(turnController.calculate(getAbsoluteEncoderRad(), targetAngle));
    }

    public void stop(){
        drive.set(0);
        turn.set(0);
    }
}
