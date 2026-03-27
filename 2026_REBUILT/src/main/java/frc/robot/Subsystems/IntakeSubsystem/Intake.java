package frc.robot.Subsystems.IntakeSubsystem;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.CommandUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;


public class Intake extends SubsystemBase {
    TalonFX drive = new TalonFX(HardwareMap.intakeDriveID.id());
    TalonFX pivot = new TalonFX(HardwareMap.intakePivotID.id());
    ProfiledPIDController pid = new ProfiledPIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, 
    new TrapezoidProfile.Constraints(360, 720));
    MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

    public Intake(){
        drive.getConfigurator().apply(IntakeConstants.applyIntakeConfig());
        pivot.getConfigurator().apply(IntakeConstants.applyPivotConfig());
        motionMagic.Slot = 0;
        pivot.setPosition(0);
    }


    //returns position in rotations
    public double getPosition(){
        double position = pivot.getPosition().getValueAsDouble();
        return position;                                  
    }

    public boolean intakeDeployed(){
        double position = getPosition();
        if ((position - IntakeConstants.kIntakePosition) < 3){
            return true;
        } else {
            return false;
        }
    }

    public void setIntakePosition(){
        pivot.setPosition(0);
    }

    public Command zeroIntake(){
        return Commands.runOnce(() -> setIntakePosition());
    }

    //returns angle in degrees
    public double getAngle(){
        return getPosition() * 360;
    }

    public void setDriveVoltage(double voltage){
        drive.setVoltage(voltage);
    }

    public void setPivotVoltage(double voltage){
        pivot.setVoltage(voltage);
    }

    public Command runIntake(double voltage){
        return Commands.startEnd(()-> setDriveVoltage(voltage), () -> setDriveVoltage(0));
    }

    // Position in rotations
    public void setPosition(double position){
        pivot.setControl(motionMagic.withPosition(position));
    }

    public void runPivot(double voltage){
        pivot.setVoltage(voltage);
    }

    public Command pivotToAngle(double position){
        double driveVoltage;

        //If going up, reverse voltage
        if (position > getPivotPosition())
            driveVoltage = -IntakeConstants.kDrivePivotVoltage;
        else
            driveVoltage = IntakeConstants.kDrivePivotVoltage;

        

        return this.run(() -> {setPosition(position); setDriveVoltage(driveVoltage); System.out.println("running");})
        .onlyWhile(() -> Math.abs(position - getPivotPosition()) > IntakeConstants.kPivotTheshold);
    }

    //pass false to extake
    public Command intake(boolean intake){
        return pivotToAngle(IntakeConstants.kIntakePosition).andThen(
            runIntake(IntakeConstants.kIntakeVoltage * ((intake) ? 1 : -1))
            .alongWith(this.runOnce(() -> setPivotVoltage(IntakeConstants.kPivotHoldsVoltage)))
        );
    }

    public double getPivotPosition(){
        return pivot.getPosition().getValueAsDouble();
    }

    public Command movePivot(double voltage){
        return this.startEnd(() -> runPivot(voltage), () -> runPivot(0));
    }

    public Command runToPosition(double position){
        return Commands.runOnce(() -> setPosition(position));
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Position", getPosition());
        SmartDashboard.putNumber("Intake Angle", getAngle());
        SmartDashboard.putBoolean("Intake Deployed", intakeDeployed());
    }
}
