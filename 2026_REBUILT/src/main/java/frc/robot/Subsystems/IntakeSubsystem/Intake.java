package frc.robot.Subsystems.IntakeSubsystem;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;


public class Intake extends SubsystemBase {
    TalonFX drive = new TalonFX(HardwareMap.intakeDriveID.id());
    TalonFX pivot = new TalonFX(HardwareMap.intakePivotID.id());

    MotionMagicVoltage pivotController = new MotionMagicVoltage(0);

    public Intake(){
        drive.getConfigurator().apply(IntakeConstants.applyIntakeConfig());
        pivot.getConfigurator().apply(IntakeConstants.applyPivotConfig());
        pivotController.Slot = 0;

        pivot.setPosition(0); //Zeroing pivot
    }

    public boolean intakeDeployed(){
        return Math.abs(getPosition() - IntakeConstants.kIntakePosition) < 3;
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromRotations(getPosition() * IntakeConstants.kGearRatio);
    }

    public double getPosition(){
        return pivot.getPosition().getValueAsDouble();                                
    }

    public void setDriveVoltage(double voltage){
        drive.setVoltage(voltage);
    }

    public void setPivotVoltage(double voltage){
        pivot.setVoltage(voltage);
    }

    public Command runIntake(double voltage){
        return this.startEnd(()-> setDriveVoltage(voltage), () -> setDriveVoltage(0));
    }

    //Position in rotations
    public void setPosition(double position){
        pivot.setControl(pivotController.withPosition(position));
    }

    public Command runToPosition(double position){
        return Commands.runOnce(() -> setPosition(position));
    }

    public Command manualPivotVoltage(double voltage){
        return Commands.startEnd(() -> setPivotVoltage(voltage), () -> setPivotVoltage(0));
    }

    //Commands -------------------

    //Deploys and runs intake at given velocity
    //Does NOT return intake to stowing position automatically
    public Command deployAndIntake(double voltage){
        return this.startEnd(
            () -> {
                setPosition(IntakeConstants.kIntakePosition);
                setDriveVoltage(voltage);
            },
            () -> setDriveVoltage(0)
        );
    }

    public Command setPositionCommand(double position){
        return this.runOnce(() -> setPosition(position));
    }

    //period is the time to go up and back down 
    public Command agitate(double firstPosition, double secondPosition, double periodSeconds){
        return Commands.repeatingSequence(
            setPositionCommand(firstPosition),
            Commands.waitSeconds (periodSeconds/2),
            setPositionCommand(secondPosition),
            Commands.waitSeconds(periodSeconds/2)
        );
    }

    public Command agitate(double periodSeconds){
        return agitate(IntakeConstants.kIntakePosition, IntakeConstants.kStowOut, periodSeconds);
    }

    public Command intake(){
        return deployAndIntake(IntakeConstants.kIntakeVoltage);
    }

    public Command extake(){
        return deployAndIntake(-IntakeConstants.kIntakeVoltage);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Position", getPosition());
        SmartDashboard.putBoolean("Intake Deployed", intakeDeployed());
    }
}
