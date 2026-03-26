// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ShooterSubsystem.Shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;
import edu.wpi.first.util.datalog.DoubleLogEntry;

public class Shooter extends SubsystemBase {
  
  Timer timer = new Timer();
  Timer time = new Timer();
  boolean ballShot;
  public static double ballsShot = 0;
  double rpsIncriment;

  TalonFX rightMotor = new TalonFX(HardwareMap.shootLeaderID.id());
  TalonFX leftMotor = new TalonFX(HardwareMap.shootFollowerID.id());

  MotionMagicVelocityVoltage mmVelocity = new MotionMagicVelocityVoltage(0);
  double targetRPS = 0;

  AnalogInput lightSensor = new AnalogInput(0);

  PIDController pidController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);

  DoubleLogEntry rpmLog;

  /** Creates a new Shooter. */

  
  public Shooter() {
    rightMotor.getConfigurator().apply(ShooterConstants.applyRightMotorConfig());
    leftMotor.getConfigurator().apply(ShooterConstants.applyLeftMotorConfig());

    //left motor follows right motor
    leftMotor.setControl(new Follower(rightMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    mmVelocity.Slot = 0;
    rpsIncriment = 0;
  
         
  // DataLogManager.start();
  // DataLog log = DataLogManager.getLog();
  // rpmLog = new DoubleLogEntry(log, "/my/double");
}


public void setVoltage(double voltage){
  VoltageOut output = new VoltageOut(voltage).withEnableFOC(true);
  rightMotor.setControl(output);
}

public Command applyVoltage(double voltage){
  return Commands.runOnce(() -> setVoltage(voltage));
}

public double getRPS(){
  return rightMotor.getVelocity().getValueAsDouble();
}

public double getTangentialVelocity(){
  double radiansPerMinute = getRPS() * 2 * Math.PI;           // radians per second
  return radiansPerMinute * ShooterConstants.kFlywheelRadius;// tangential velocity (meters / second)
}

public boolean readyToShoot(){
  //use defualt tolerances
  return Math.abs(getRPS() - targetRPS) < ShooterConstants.kRPSTolerance;
}

public double getRPSIncriment(){
  return rpsIncriment;
}

//need to add in piece wise best fit polynomial for RPM target RPM estimation
public void setRPS(double targetVelocity){
  targetRPS = targetVelocity;// + rpsIncriment;
  rightMotor.setControl(mmVelocity.withVelocity(targetVelocity)); 
}

public void spinDown(){
  targetRPS = 0;
  setVoltage(0);
}

public Command setFlywheelRPS(double targetRPM){
  return Commands.startEnd(() -> setRPS(targetRPM), () -> setRPS(0));
}

public void setTorque(double torque){
  double pidOutput = pidController.calculate(torque, rightMotor.getStatorCurrent().getValueAsDouble()) + ShooterConstants.kFrictionFF;
  TorqueCurrentFOC motorTorque = new TorqueCurrentFOC(pidOutput);
  rightMotor.setControl(motorTorque);
}

public Command setFlywheelTorque(double torque){
  return Commands.runOnce(() -> setTorque(torque));
}

public double lightSensorReading(){
  return lightSensor.getVoltage();
}

public double getFlywheelStatorCurrent(){
  return rightMotor.getStatorCurrent().getValueAsDouble();
}

@Override
public void periodic() {
    SmartDashboard.putNumber("Shooter RPS", getRPS());
    SmartDashboard.putNumber("Target RPS", targetRPS);
    SmartDashboard.putBoolean("Shooter ready to shoot", readyToShoot());
  }
}
