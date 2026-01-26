package frc.robot.IntakeSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    TalonFX intake = new TalonFX(IntakeConstants.kIntakeCanID);
    TalonFX pivot = new TalonFX(IntakeConstants.kPivotCanID);
    PIDController pid = new PIDController(IntakeConstants.kP,IntakeConstants.kI,IntakeConstants.kD);

    
    public Intake(){
        intake.getConfigurator().apply(IntakeConstants.applyIntakeConfig());
        pivot.getConfigurator().apply(IntakeConstants.applyPivotConfig());

        pivot.setPosition(0);
        pivot.setVoltage(pid.calculate(getPosition(), 0));

    }

    //returns anlge of pivot motor in degrees
    public double getPosition(){
        double angle = pivot.getPosition().getValueAsDouble();//in rotations
        return angle * 360;                                   //degrees
    }

    public void setVoltage(double voltage){
        intake.setVoltage(voltage);
    }

    public Command runIntake(double voltage){
        return Commands.runOnce(()-> setVoltage(voltage));
    }

    public void setPosition(double position){
        pivot.setVoltage(pid.calculate(getPosition(), position));
    }

    public Command runToPosition(double position){
        return Commands.runOnce(() -> setPosition(position));
    }

    


}
