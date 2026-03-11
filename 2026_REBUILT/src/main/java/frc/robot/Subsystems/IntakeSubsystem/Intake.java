package frc.robot.Subsystems.IntakeSubsystem;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;


public class Intake extends SubsystemBase {
    TalonFX intake = new TalonFX(HardwareMap.intakeDriveID);
    TalonFX pivot = new TalonFX(HardwareMap.intakePivotID);
    ProfiledPIDController pid = new ProfiledPIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, 
    new TrapezoidProfile.Constraints(360, 720));
    MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

    public Intake(){
        intake.getConfigurator().apply(IntakeConstants.applyIntakeConfig());
        pivot.getConfigurator().apply(IntakeConstants.applyPivotConfig());
        motionMagic.Slot = 0;
        //pivot.setPosition(0);
    }


    //returns position in rotations
    public double getPosition(){
        double position = pivot.getPosition().getValueAsDouble();
        return position;                                  
    }

    //returns angle in degrees
    public double getAngle(){
        double position = pivot.getPosition().getValueAsDouble();
        return position * 360;
    }

    public void setVoltage(double voltage){
        intake.setVoltage(voltage);
    }

    public Command runIntake(double voltage){
        return Commands.startEnd(()-> setVoltage(voltage), () -> setVoltage(0));
    }

    // Position in rotations
    public void setPosition(double position){
        pivot.setControl(motionMagic.withPosition(position));
    }

    public void runPivot(double voltage){
        pivot.setVoltage(voltage);
    }

    public Command movePivot(double voltage){
        return Commands.startEnd(() -> runPivot(voltage), () -> runPivot(0));
    }

    public Command runToPosition(double position){
        return Commands.runOnce(() -> setPosition(position));
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Position", getPosition());
    }
}
