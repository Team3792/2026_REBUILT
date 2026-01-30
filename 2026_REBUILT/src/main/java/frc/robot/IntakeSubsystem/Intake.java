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
    ProfiledPIDController pid = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

    public Intake(){
        intake.getConfigurator().apply(IntakeConstants.applyIntakeConfig());
        pivot.getConfigurator().apply(IntakeConstants.applyPivotConfig());

        pivot.setPosition(0);
        pid.setGoal(0);
        pid.reset(intakeAngle());

    }

    // return angle of intake in degrees
    // where an angle of 0 is vertical
    public double intakeAngle(){
        return MathUtil.inputModulus((pivot.getPosition().getValueAsDouble() - IntakeConstants.kEncoderOffset) * 360,0,360);
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
        pid.setGoal(position);
    }

    public Command runToPosition(double position){
        return Commands.runOnce(() -> setPosition(position));
    }

    


}
