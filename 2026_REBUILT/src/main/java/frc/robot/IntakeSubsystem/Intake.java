package frc.robot.IntakeSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShooterSubsystem.Hood.HoodConstants;

public class Intake extends SubsystemBase {
    TalonFX intake = new TalonFX(IntakeConstants.kIntakeCanID);
    TalonFX pivot = new TalonFX(IntakeConstants.kPivotCanID);
    DutyCycleEncoder encoder = new DutyCycleEncoder(IntakeConstants.kEncoderCanID);
    PIDController pid = new PIDController(IntakeConstants.kP,IntakeConstants.kI,IntakeConstants.kD);

    
    public Intake(){
        intake.getConfigurator().apply(IntakeConstants.applyIntakeConfig());
        pivot.getConfigurator().apply(IntakeConstants.applyPivotConfig());

        pivot.setPosition(0);
        pivot.setVoltage(pid.calculate(intakeAngle(), 0));

    }

    // return angle of intake in degrees
    // where an angle of 0 is vertical
    public double intakeAngle(){
        return MathUtil.inputModulus((encoder.get() - IntakeConstants.kEncoderOffset) * 360,0,360);
    }

    //returns anlge of pivot motor in degrees
    // public double getPosition(){
    //     double angle = pivot.getPosition().getValueAsDouble();//in rotations
    //     return angle * 360;                                   //degrees
    // }

    public void setVoltage(double voltage){
        intake.setVoltage(voltage);
    }

    public Command runIntake(double voltage){
        return Commands.runOnce(()-> setVoltage(voltage));
    }

    public void setPosition(double position){
        pivot.setVoltage(pid.calculate(intakeAngle(), position));
    }

    public Command runToPosition(double position){
        return Commands.runOnce(() -> setPosition(position));
    }

    


}
