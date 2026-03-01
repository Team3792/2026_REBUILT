package frc.robot.Subsystems.SwerveSubsystem;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.SwerveSubsystem.ModuleConstants.ModuleConfig;

public class SwerveModule{
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
        encoder.getConfigurator().apply(ModuleConstants.getEncoderConfig(config.encoderOffset())); //Fix offset

        driveController = new VelocityVoltage(0).withSlot(0);
        turnController = new PIDController(6, 0, 0.05);
        turnController.enableContinuousInput(-Math.PI, Math.PI);   

        drive.setPosition(0); 
        resetTurnEncoder();
    }

    public void resetEncoders(){
        resetTurnEncoder();
        resetDriveEncoder();
        resetCancoder();
    }


    public void resetTurnEncoder(){
        System.out.println("Resetting encoder" + name);
        double position = encoder.getPosition().getValueAsDouble() * ModuleConstants.kTurnRatio;
        turn.setPosition(position);
       // turn.setPosition(0);
    }

    public void zeroTurnEncoder(){
        turn.setPosition(0);
    }

    public void resetDriveEncoder(){
        drive.setPosition(0);
    }

    public void resetCancoder(){
        //encoder.setPosition(0);
    }

    public void driveVoltage(double voltage){
        drive.setVoltage(voltage);
        double turnVoltage = turnController.calculate(getTurnPosition().getRadians(), 0);
        turn.setVoltage(turnVoltage);
    }

    public Rotation2d getTurnPosition(){
        return new Rotation2d(turn.getPosition().getValueAsDouble() * ModuleConstants.kRadiansPerMotorRotation);
    }

    public double getTurnPositionDegrees(){
        return encoder.getPosition().getValueAsDouble() * 360.0;
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), getTurnPosition());
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

    public void setState(SwerveModuleState state){
        //System.out.println(state.speedMetersPerSecond);
        if(Math.abs(state.speedMetersPerSecond) > 0.05){
            state.optimize(getTurnPosition());

            drive.setControl(driveController.withVelocity(state.speedMetersPerSecond / ModuleConstants.kMetersPerMotorRotation));
            
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


    public void printEncoders() {
        SmartDashboard.putNumber(name + "Cancoder Position", encoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber(name + "Turn Encoder Position", turn.getPosition().getValueAsDouble());// * ModuleConstants.kRadiansPerRotation);
        SmartDashboard.putNumber(name + "Drive Encoder Position", getDrivePosition());
    }
}
