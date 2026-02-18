// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ShooterSubsystem.Shooter;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
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
import frc.robot.SwerveSubsystem.Swerve;

public class Shooter extends SubsystemBase {
  
  Timer timer = new Timer();
  Timer time = new Timer();
  boolean ballShot;
  public static double ballsShot = 0;

  TalonFX rightMotor = new TalonFX(HardwareMap.shootLeaderID);
  TalonFX leftMotor = new TalonFX(HardwareMap.shootFollowerID);

  AnalogInput lightSensor = new AnalogInput(0);

  PIDController pidController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);


  /** Creates a new Shooter. */

  
  public Shooter() {
    rightMotor.getConfigurator().apply(ShooterConstants.applyRightMotorConfig());
    leftMotor.getConfigurator().apply(ShooterConstants.applyLeftMotorConfig());

    

    //left motor follows right motor
    //aligned means on same shaft; may need to change to opposed
    leftMotor.setControl(new Follower(rightMotor.getDeviceID(), MotorAlignmentValue.Aligned));
         
}


public void setVoltage(double voltage){
  VoltageOut output = new VoltageOut(12).withEnableFOC(true);
  rightMotor.setControl(output);
 
 
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
  return radiansPerMinute * ShooterConstants.kFlywheelRadius;// tangential velocity (meters / minute)
}

//need to add in piece wise best fit polynomial for RPM target RPM estimation

public void setRPM(double targetVelocity){
  double pidOutput = pidController.calculate(targetVelocity,getRPM()) + ShooterConstants.kFrictionFF;
  VoltageOut motorVoltage = new VoltageOut(pidOutput).withEnableFOC(true);
  rightMotor.setControl(motorVoltage);
}

public Command setFlywheelRPM(double targetRPM){
  return Commands.runOnce(() -> setRPM(targetRPM));
}

public void setTorque(double torque){
  double pidOutput = pidController.calculate(torque, rightMotor.getStatorCurrent().getValueAsDouble()) + ShooterConstants.kFrictionFF;
  TorqueCurrentFOC motorTorque = new TorqueCurrentFOC(pidOutput);
  rightMotor.setControl(motorTorque);
}

public Command setFlywheelTorque(double torque){
  return Commands.runOnce(() -> setTorque(torque));
}

public double targetFlywheelVelocity(Swerve swerve, Pose2d hubPose){

  Pose2d currentPos = swerve.getPose();
  Translation2d robotToHubPose = hubPose.getTranslation().minus(currentPos.getTranslation());

    //double robotToHubDistance = Math.hypot((robotToHubPose.getX(),robotToHubPose.getY());
    //velocity of robot relative to field; in x direction
  double fieldRelativeVelocityX = swerve.getFieldRelativeSpeeds()[0];
  double robotToHubDistance = robotToHubPose.getDistance(currentPos.getTranslation());

  double targetRPM = 0;

  for(int i = 1; i < ShooterConstants.distanceVSRPM[0].length; i++){
      if(robotToHubDistance > ShooterConstants.distanceVSRPM[0][i]){
        //return target flywheel RPM
        targetRPM = ShooterConstants.distanceVSRPM[1][i-1];
        break;
      }
    
    }

    return targetRPM;

}

public boolean ballInShooter(){
  return lightSensor.getVoltage() > ShooterConstants.kLightSensorThreshold;
}

//time it take for ball to pass through the light sensor detection range
public double ballShootTime(){
  return ShooterConstants.lightSensorDetectionRange / getTangentialVelocity();
}

//if ball detected is in shooter, and ball is gone after the time it takes to shoot 1 ball; returns true
public boolean ballShot(){
  if(ballInShooter()){
    timer.start();
    if(timer.hasElapsed(ballShootTime())){
      ballShot = !ballInShooter();
    }
  }
  timer.reset();
  return ballShot;
}

public double flywheelStatorCurrent(){
  return rightMotor.getStatorCurrent().getValueAsDouble();
}
//return true if flywheel current < current of flywheel under load of a ball && if there has been a decrease if flywheel RPMs
public boolean ballShotCurrentDetection(Swerve swerve, Pose2d hubPose){
  return (flywheelStatorCurrent() < ShooterConstants.kFlywheelBallLoadCurrent) 
      && (getRPM() < targetFlywheelVelocity(swerve, hubPose) );
}

public double ballsShot(){
  time.reset();
  time.start();
  if(ballShot()){
    time.hasElapsed(ShooterConstants.kballSeperationTime);
    if(!ballShot){
      ballsShot += 1;
    }
  }
  return ballsShot;
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ShooterSubsystems/Shooter", getRPM());
    SmartDashboard.putNumber("ShooterSubsystems/Shooter", getTangentialVelocity());
  }
}
