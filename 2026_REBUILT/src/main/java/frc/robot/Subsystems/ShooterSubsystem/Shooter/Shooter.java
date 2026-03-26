// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ShooterSubsystem.Shooter;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;
import frc.robot.Managers.FieldManager;
import frc.robot.Subsystems.SwerveSubsystem.Swerve;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class Shooter extends SubsystemBase {
  
  Timer timer = new Timer();
  Timer time = new Timer();
  boolean ballShot;
  public static double ballsShot = 0;
  double rpsIncriment;

  TalonFX rightMotor = new TalonFX(HardwareMap.shootLeaderID.id());
  TalonFX leftMotor = new TalonFX(HardwareMap.shootFollowerID.id());

  MotionMagicVelocityVoltage mmVelocity = new MotionMagicVelocityVoltage(0);

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
  int rps = (int) rightMotor.getVelocity().getValueAsDouble();
  return rps;
}

public double getTangentialVelocity(){
  double radiansPerMinute = getRPS() * 2 * Math.PI;           // radians per second
  return radiansPerMinute * ShooterConstants.kFlywheelRadius;// tangential velocity (meters / second)
}

public double getTargetRPS(double distance){
  //Linear search (assume already sorted calibrated shots) --> could switch to binary search
  for (int i = 0; i < ShooterConstants.kCalibratedShots.length - 1; i++){
    double calibratedDistance = ShooterConstants.kCalibratedShots[i].distanceMeters();
    if (distance > calibratedDistance){
      //Linear interpolation between two 
      double t = (distance - calibratedDistance) / (ShooterConstants.kCalibratedShots[i].distanceMeters() - calibratedDistance);
      return ShooterConstants.kCalibratedShots[i].omegaRPS() * (1 - t)
                 + ShooterConstants.kCalibratedShots[i].omegaRPS() *(t); 
    }
  }
  return 0; //Out of range
}



public boolean isAtTargetRPS(double targetRPS){
  if (Math.abs(getRPS() - targetRPS) < 4){
    return true;
  } else {
    return false;
  }
}

public double getRPSIncriment(){
  return rpsIncriment;
}

public void incrimentRPS(){
  rpsIncriment += 2;
}

public void decrimentRPS(){
  rpsIncriment--;
}

public Command increaseRPS(){
  return Commands.runOnce(() -> incrimentRPS());
}

public Command decreaseRPS(){
  return Commands.runOnce(() -> decrimentRPS());
}

//need to add in piece wise best fit polynomial for RPM target RPM estimation
public void setRPS(double targetVelocity){
  // double pidOutput = pidController.calculate(targetVelocity,getRPM()) + ShooterConstants.kFrictionFF;
  // VoltageOut motorVoltage = new VoltageOut(pidOutput).withEnableFOC(true);
  // rightMotor.setControl(motorVoltage);

  // mm.withVelocity in rps
  rightMotor.setControl(mmVelocity.withVelocity(targetVelocity)); 
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

// public double targetFlywheelVelocity(Swerve swerve, Pose2d hubPose){

//   Pose2d currentPos = swerve.getPose();
//   Translation2d robotToHubPose = hubPose.getTranslation().minus(currentPos.getTranslation());

//     //double robotToHubDistance = Math.hypot((robotToHubPose.getX(),robotToHubPose.getY());
//     //velocity of robot relative to field; in x direction
//   double fieldRelativeVelocityX = swerve.getFieldRelativeSpeeds()[0];
//   double robotToHubDistance = robotToHubPose.getDistance(currentPos.getTranslation());

//   double targetRPM = 0;

//   for(int i = 1; i < ShooterConstants.distanceVSRPM[0].length; i++){
//       if(robotToHubDistance > ShooterConstants.distanceVSRPM[0][i]){
//         //return target flywheel RPM
//         targetRPM = ShooterConstants.distanceVSRPM[1][i-1];
//         break;
//       }
    
//     }

//     return targetRPM;

// }

public double lightSensorReading(){
  return lightSensor.getVoltage();
}

public double flywheelStatorCurrent(){
  return rightMotor.getStatorCurrent().getValueAsDouble();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPS", getRPS());
    SmartDashboard.putNumber("Shooter Tangential Velocity", getTangentialVelocity());

    //rpmLog.append(getRPM());
  }
}
