package frc.robot.SwerveSubsystem;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveSubsystem.ModuleConstants.ModuleConfig;

public class SwerveModule extends SubsystemBase{
    TalonFX drive, turn;
    CANcoder encoder;
    PIDController turnController;
    VelocityVoltage driveController;
    SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(0));

    String name;

    public SwerveModule(ModuleConfig config, String name) {
        drive = new TalonFX(config.driveAddress());
        turn = new TalonFX(config.turnAddress());
        encoder = new CANcoder(config.encoderAddress());
        this.name = name;
        
        drive.getConfigurator().apply(ModuleConstants.getDriveConfig());
        turn.getConfigurator().apply(ModuleConstants.getTurnConfig());
        encoder.getConfigurator().apply(ModuleConstants.getEncoderConfig(0.0)); //Fix offset

        driveController = new VelocityVoltage(0).withSlot(0);
        turnController = new PIDController(4, 0, 0.05);
        turnController.enableContinuousInput(-Math.PI, Math.PI);   

        drive.setPosition(0);  
        resetTurnEncoder();
    }


    public void resetTurnEncoder(){
        double position = encoder.getPosition().getValueAsDouble() * ModuleConstants.kTurnRatio;
        turn.setPosition(position);
    }

    public void driveVoltage(double voltage){
        drive.setVoltage(voltage);
        double turnVoltage = turnController.calculate(getTurnPosition().getRadians(), 0);
        turn.setVoltage(turnVoltage);
    }

    public Rotation2d getTurnPosition(){
        return new Rotation2d(turn.getPosition().getValueAsDouble() * ModuleConstants.kRadiansPerRotation);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), getTurnPosition());
    }
    
    public double getDrivePosition(){
        return drive.getPosition().getValueAsDouble() * ModuleConstants.kMetersPerRotation;
    }

    public double getDriveVelocity(){
        return drive.getVelocity().getValueAsDouble() * ModuleConstants.kMetersPerRotation;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), getTurnPosition());
    }

    public void setState(SwerveModuleState state){
        System.out.println(state.speedMetersPerSecond);
        if(Math.abs(state.speedMetersPerSecond) > 0.1){
            state.optimize(getTurnPosition());

            drive.setControl(driveController.withVelocity(state.speedMetersPerSecond / ModuleConstants.kMetersPerRotation));
            
            double turnVoltage = turnController.calculate(getTurnPosition().getRadians(), state.angle.getRadians());
            turn.setVoltage(turnVoltage);
        } else {
            stop();
        }
    }

    public void stop(){
        turn.setVoltage(0);
        drive.setVoltage(0);
    }

    // public double getAbsoluteEncoderRad() {
    //     double angle = (encoder.getAbsolutePosition().getValueAsDouble()) * 2.0 * Math.PI;
    //     return angle;
    // }

}
