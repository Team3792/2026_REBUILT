package frc.robot.Subsystems.IntakeSubsystem;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.HardwareMap;

public class Intake extends SubsystemBase {
    TalonFX driveLeader = new TalonFX(HardwareMap.intakeDriveLeftID.id());
    TalonFX driveFollower = new TalonFX(HardwareMap.intakeDriveRightID.id());
    TalonFX pivot = new TalonFX(HardwareMap.intakePivotID.id());

    MotionMagicVoltage pivotController = new MotionMagicVoltage(0);

    public Intake(){
        driveLeader.getConfigurator().apply(IntakeConstants.applyLeftIntakeConfig());
        driveFollower.getConfigurator().apply(IntakeConstants.applyRightIntakeConfig());
        pivot.getConfigurator().apply(IntakeConstants.applyPivotConfig());
        pivotController.Slot = 0;  
        pivot.setPosition(0);
        //driveLeader is the leader, leader motor is in the first param of new Follower
        driveFollower.setControl(new Follower(driveLeader.getDeviceID(), MotorAlignmentValue.Opposed));
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
        driveLeader.setVoltage(voltage);
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

    public Command runToPosition(double position, double tolerance){
        return Commands.runOnce(() -> setPosition(position)).
        andThen(Commands.waitUntil(
            () -> Math.abs(getPosition() - position) < tolerance)
        );
    }

    public Command runToPosition(double position){
        return runToPosition(position, 1);
    }

    public Command manualPivotVoltage(double voltage){
        return Commands.startEnd(() -> setPivotVoltage(voltage), () -> setPivotVoltage(0));
    }

    //Commands -------------------

    //Deploys and runs intake at given velocity
    //Does NOT return intake to stowing position automatically
    public Command deployAndIntake(double voltage){
        return runToPosition(IntakeConstants.kIntakePosition).andThen(this.startEnd(() -> setDriveVoltage(voltage), () -> setDriveVoltage(0)));
    }

    public Command setPositionCommand(double position){
        return this.runOnce(() -> setPosition(position));
    }

    //period is the time to go up and back down, ending on second position
    public Command agitate(double firstPosition, double secondPosition, double periodSeconds){
        return Commands.repeatingSequence(
            setPositionCommand(firstPosition),
            Commands.waitSeconds (periodSeconds/2),
            setPositionCommand(secondPosition),
            Commands.waitSeconds(periodSeconds/2)
        ).finallyDo((boolean b) -> setPositionCommand(secondPosition));
    }

    public Command agitate(double periodSeconds){
        return agitate(IntakeConstants.kIntakePosition, IntakeConstants.kFramePerimStow, periodSeconds);
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
