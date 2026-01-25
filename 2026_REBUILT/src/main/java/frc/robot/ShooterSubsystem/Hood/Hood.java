// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ShooterSubsystem.Hood;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */
  SparkMax hood = new SparkMax(HoodConstants.kHoodCANID, MotorType.kBrushless);
  DutyCycleEncoder encoder = new DutyCycleEncoder(HoodConstants.kHoodCANID);
  ProfiledPIDController pidController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(5, 10));
  
  public Hood() {
    pidController.enableContinuousInput(0, 360);
    hood.configure(HoodConstants.configHood(), com.revrobotics.ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public double getEncoderPositon() {
    return encoder.get();
  }

  public void setAngle(double angle) {
    pidController.setGoal(angle);
    hood.set(pidController.calculate(getEncoderPositon(), angle) + HoodConstants.frictionFF);
  }

  public Command setPosition(double angle){
    return Commands.runOnce(() -> setAngle(angle));
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
