package frc.robot.Subsystems.ClimbSubsytem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;

public class Climb extends SubsystemBase {

    TalonFX climbMotor = new TalonFX(HardwareMap.climbID);

    public Climb() {
        climbMotor.getConfigurator().apply(ClimbConstants.getConfig());
    }

    public void setVoltage(double voltage){
        climbMotor.setVoltage(voltage);
    }

    public Command climbUpCommand(){
        return Commands.startEnd(() -> setVoltage(ClimbConstants.kClimbVoltage), () -> setVoltage(0));
    }

    public Command climbDownCommand(){
        return Commands.startEnd(() -> setVoltage(-ClimbConstants.kClimbVoltage), () -> setVoltage(0));
    }
}

