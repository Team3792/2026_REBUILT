// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ShooterSubsystem.Shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;

public class Shooter extends SubsystemBase {

  TalonFX rightMotor = new TalonFX(HardwareMap.shootLeaderID);
  TalonFX leftMotor = new TalonFX(HardwareMap.shootFollowerID);


  ProfiledPIDController pidController = new ProfiledPIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD, 
    new TrapezoidProfile.Constraints(20, 10));


  /** Creates a new Shooter. */
  public Shooter() {
    rightMotor.getConfigurator().apply(ShooterConstants.applyRightMotorConfig());
    leftMotor.getConfigurator().apply(ShooterConstants.applyLeftMotorConfig());

    //left motor follows right motor
    //aligned means on same shaft; may need to change to opposed
    leftMotor.setControl(new Follower(rightMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    
}

public void setVoltage(double voltage){
  rightMotor.setVoltage(voltage);
}

public Command applyVoltage(double voltage){
  return Commands.runOnce(() -> setVoltage(voltage));
}

public double getRPM(){
   double rps = rightMotor.getVelocity().getValueAsDouble();
  return rps * 60;
}

public double getTangentialVelocity(){
  double radiansPerMinute = getRPM() * 2 * Math.PI;           // radians per minute
  return radiansPerMinute * ShooterConstants.flywheelRadius;// tangential velocity (meters / minute)
}

//need to add in piece wise best fit polynomial for RPM target RPM estimation

public void setRPM(double targetVelocity){
  pidController.setGoal(targetVelocity);
  double pidOutput = pidController.calculate(getRPM());
  rightMotor.setVoltage(pidOutput + ShooterConstants.frictionFF);
}

public Command setAngularVelocity(double targetRPM){
  return Commands.runOnce(() -> setRPM(targetRPM));
}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ShooterSubsystems/Shooter", getRPM());
    SmartDashboard.putNumber("ShooterSubsystems/Shooter", getTangentialVelocity());
  }
}
