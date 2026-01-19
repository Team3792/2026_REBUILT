package frc.robot.ClimbSubsytem;

public class Climb extends SubsystemBase {

    TalonFX leftClimbMotor = new TalonFX(ClimbConstants.kleftClimbID);
    TalonFX rightClimbMotor = new TalonFX(ClimbConstants.krightClimbID);

    public Climb() {
        leftClimbMotor.getConfigurator().apply(ClimbConstants.getConfig());
        rightClimbMotor.getCOnfigurator().apply(ClimbConstants.getConfig());

        var limitConfig = new HardwareLimitSwitchConfig();
        limitConfig.ForwardLimitSource = ForwardLimitSourceValue.TalonFX; //Make sure this is the correct syntax

        leftClimbMotor.getConfigurator().apply(limitConfig);
        rightClimbMotor.getConfigurator().apply(limitConfig);
    }

    public void setVoltage(double voltage)){
        leftClimbMotor.setVoltage(voltage);
        rightClimbMotor.setVoltage(voltage);
    }

    public Command climbUpCommand(){
        return Commands.runOnce(() -> setVoltge(ClimbConstants.kClimbVoltage));
    }

    public Command climbDownCommand(){
        return Commands.runOnce(() -> setVoltage(-ClimbConstants.kClimbVoltage))
    }
}

