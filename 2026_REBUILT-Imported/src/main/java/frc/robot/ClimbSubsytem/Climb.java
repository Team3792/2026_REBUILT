package frc.robot.ClimbSubsytem;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {

    TalonFX leftClimbMotor = new TalonFX(ClimbConstants.kLeftClimbID);
    TalonFX rightClimbMotor = new TalonFX(ClimbConstants.kRightClimbID);

    public Climb() {
        leftClimbMotor.getConfigurator().apply(ClimbConstants.getConfig());
        rightClimbMotor.getConfigurator().apply(ClimbConstants.getConfig());

        var leftLimitConfig = new HardwareLimitSwitchConfigs();
        leftLimitConfig.ForwardLimitSource = ForwardLimitSourceValue.RemoteTalonFX;
        leftLimitConfig.ForwardLimitRemoteSensorID = leftClimbMotor.getDeviceID();
        
        var rightLimitConfig = new HardwareLimitSwitchConfigs();
        rightLimitConfig.ForwardLimitSource = ForwardLimitSourceValue.RemoteTalonFX;
        rightLimitConfig.ForwardLimitRemoteSensorID = rightClimbMotor.getDeviceID();

        leftClimbMotor.getConfigurator().apply(leftLimitConfig);
        rightClimbMotor.getConfigurator().apply(rightLimitConfig);
    }

    public void setVoltage(double voltage){
        leftClimbMotor.setVoltage(voltage);
        rightClimbMotor.setVoltage(voltage);
    }

    public Command climbUpCommand(){
        return Commands.runOnce(() -> setVoltage(ClimbConstants.kClimbVoltage));
    }

    public Command climbDownCommand(){
        return Commands.runOnce(() -> setVoltage(-ClimbConstants.kClimbVoltage));
    }
}

