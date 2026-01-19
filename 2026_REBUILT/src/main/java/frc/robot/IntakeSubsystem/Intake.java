package frc.robot.IntakeSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Intake extends SubsystemBase {
    TalonFX intake = new TalonFX(intakeConstants.kIntakeCanID);
    TalonFX pivot = new TalonFX(intakeConstants.kPivotCanID);
    PIDController pid = new PIDController(0,0,0); //TO-DO; Tune PID

    
    public Intake(){
        intake.getConfigurator().apply(IntakeConstants.applyIntakeConfig());
        pivot.getConfigurator().apply(IntakeConstants.applyPivotConfig());

        pivot.setPosition(0);
        pid.setGoal(0);
    }

    public void setVoltage(double voltage){
        intake.setVoltage(voltage);
    }

    public Command runIntake(double voltage){
        return Commands.runOnce(()-> setVoltage(voltage));
    }

    public void setPosition(double position){
        pid.setGoal(position)
    }

    public Command runToPosition(double position){
        return Commands.runOnce(() -> setPosition(position));
    }

    


}
