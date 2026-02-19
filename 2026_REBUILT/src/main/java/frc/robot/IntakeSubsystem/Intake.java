package frc.robot.IntakeSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;


public class Intake extends SubsystemBase {
    TalonFX intake = new TalonFX(HardwareMap.intakeDriveID);
    TalonFX pivot = new TalonFX(HardwareMap.intakePivotID);
    PIDController pid = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);

    public Intake(){
        intake.getConfigurator().apply(IntakeConstants.applyIntakeConfig());
        pivot.getConfigurator().apply(IntakeConstants.applyPivotConfig());

        pivot.setPosition(0);

    }

    // return angle of intake in degrees
    // where an angle of 0 is vertical
    public double intakeAngle(){
        return MathUtil.inputModulus((pivot.getPosition().getValueAsDouble() - IntakeConstants.kEncoderOffset) * 360,0,360);
    }

    //returns angle of pivot motor in degrees
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
        double pidOutput = pid.calculate(getPosition(), position) 
                         + IntakeConstants.kFrictionFF 
                         + IntakeConstants.GravityFF(getPosition());
    }

    public Command runToPosition(double position){
        return Commands.runOnce(() -> setPosition(position));
    }

    


}
