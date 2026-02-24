// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ShooterSubsystem.Hood;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */
  SparkMax hood = new SparkMax(HardwareMap.hoodID, MotorType.kBrushless);
  DutyCycleEncoder encoder = new DutyCycleEncoder(HardwareMap.hoodEncoderPort);
  ProfiledPIDController pidController = new ProfiledPIDController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD, 
    new TrapezoidProfile.Constraints(360, 360));
  
  public Hood() {
    pidController.enableContinuousInput(0, 360);
    hood.configure(HoodConstants.configHood(), com.revrobotics.ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pidController.enableContinuousInput(0, 360);
  }

  public void setVoltage(double voltage){
    hood.setVoltage(voltage);
  }

  public Command applyVoltage(double voltage){
    return Commands.runOnce(() -> setVoltage(voltage));
  }

  //returns rotations
  public double getEncoderRotation() {
    return encoder.get();
  }

  //returns hood positions in degrees 
  //where an angle of 0 is a ball exit angle along the horizontal axis
  //an angle of 90 is ball exit angle along the vertical axis
  public double hoodAngle(){
    return MathUtil.inputModulus((getEncoderRotation() - HoodConstants.kEncoderAngleOffset) * 360, 0, 360);
  }

  public void setAngle(double angle) {
    pidController.setGoal(angle);
    double pidOutput = pidController.calculate(hoodAngle());
    hood.set(pidOutput + HoodConstants.kFrictionFF);
  }

  public Command setPosition(double angle){
    return Commands.runOnce(() -> setAngle(angle));
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ShooterSubsystem/Hood", hoodAngle());
  }
}
