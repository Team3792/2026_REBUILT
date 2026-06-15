package frc.robot.Subsystems.SwerveSubsystem;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.SwerveSubsystem.ModuleConstants.ModuleConfig;

public class SwerveModule{
    TalonFX drive, turn;
    CANcoder encoder;
    PIDController turnController;
    VelocityVoltage driveController;
    String name;

    public SwerveModule(ModuleConfig config, String name) {
        drive = new TalonFX(config.driveAddress().id());
        turn = new TalonFX(config.turnAddress().id());
        encoder = new CANcoder(config.encoderAddress().id());
        this.name = name;
        
        drive.getConfigurator().apply(ModuleConstants.getDriveConfig());
        turn.getConfigurator().apply(ModuleConstants.getTurnConfig());
        encoder.getConfigurator().apply(ModuleConstants.getEncoderConfig(config.encoderOffset())); //Fix offset

        driveController = new VelocityVoltage(0).withSlot(0);
        //TODO: re-tune?
        turnController = new PIDController(6, 0, 0.05);
        turnController.enableContinuousInput(-Math.PI, Math.PI);   

        resetEncoders();
    }

    public void resetEncoders(){
        resetTurnEncoder();
        resetDriveEncoder();
    }

    public void resetTurnEncoder(){
        //Everything is in rotations
        double newPosition = encoder.getPosition().getValueAsDouble() * ModuleConstants.kTurnRatio;
        turn.setPosition(newPosition);
    }

    public void resetDriveEncoder(){
        drive.setPosition(0);
    }

    public Rotation2d getTurnPosition(){
        return new Rotation2d(turn.getPosition().getValueAsDouble() * ModuleConstants.kRadiansPerMotorRotation);
    }
    
    public double getDrivePosition(){
        return drive.getPosition().getValueAsDouble() * ModuleConstants.kMetersPerMotorRotation;
    }

    public double getDriveVelocity(){
        return drive.getVelocity().getValueAsDouble() * ModuleConstants.kMetersPerMotorRotation;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), getTurnPosition());
    }

     public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), getTurnPosition());
    }

    public void setState(SwerveModuleState state){
        state.optimize(getTurnPosition());
        drive.setControl(driveController.withVelocity(state.speedMetersPerSecond / ModuleConstants.kMetersPerMotorRotation));
        double turnVoltage = turnController.calculate(getTurnPosition().getRadians(), state.angle.getRadians());
        turn.setVoltage(turnVoltage);
    }

    public void stop(){
        turn.setVoltage(0);
        drive.setVoltage(0);
    }

    public void printEncoders() {
        SmartDashboard.putNumber(name + "Cancoder Position", encoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber(name + "Turn Encoder Position", turn.getPosition().getValueAsDouble());// * ModuleConstants.kRadiansPerRotation);
        SmartDashboard.putNumber(name + "Drive encoder raw", drive.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(name + "Drive Encoder Position", getDrivePosition());
    }
}
