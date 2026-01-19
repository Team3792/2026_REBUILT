package frc.robot.IntakeSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Intake {
    IntakeConstants intakeConstants = new IntakeConstants();
    TalonFX intake = new TalonFX(intakeConstants.kCanID);
    
    public Intake(){
        intake.getConfigurator().apply(intakeConstants.applyConfiguration());
    }

    public void setVoltage(double voltage){
        intake.setVoltage(voltage);
    }

    public Command runIntake(double voltage){
        return Commands.runOnce(()-> setVoltage(voltage));
    }

}
