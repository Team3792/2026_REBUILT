// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ShooterSubsystem.Hood;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;
import frc.robot.Managers.FieldManager;
import frc.robot.Subsystems.ShooterSubsystem.Shooter.Shooter;
import frc.robot.Subsystems.ShooterSubsystem.Shooter.ShooterConstants;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */
  SparkMax hood = new SparkMax(HardwareMap.hoodID, MotorType.kBrushless);
  SparkClosedLoopController controller;
  double setPoint = 12;
  double angleIncrement = 0;
  
  public Hood() {
    hood.configure(HoodConstants.configHood(), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    controller = hood.getClosedLoopController();
    zeroHood();
    //setAngle(0);
  }

  // //returns hood positions in degrees 
  // //where an angle of 0 is a ball exit angle along the horizontal axis
  // //an angle of 90 is ball exit angle along the vertical axis
  public double getAngleDegrees(){
    //return MathUtil.inputModulus((getPosition() - HoodConstants.kHoodOffset) * 360, 0, 360);
    return hood.getEncoder().getPosition() / HoodConstants.kHoodRatio / HoodConstants.kMotorRatio * 360.0;
  }

  public void setAngle(double angle) {
    setPoint = angle + angleIncrement;
    controller.setSetpoint(angle / 360.0 * HoodConstants.kHoodRatio * HoodConstants.kMotorRatio, ControlType.kPosition);
  }

  public Command incrementHoodAngle(double increment){
    return this.runOnce(() -> {
      angleIncrement += increment;
      setAngle(setPoint);
    });
  }

  public void zeroHood(){
    hood.getEncoder().setPosition(HoodConstants.kZeroAngle / 360.0 * HoodConstants.kHoodRatio * HoodConstants.kMotorRatio);
  }

  public Command setHoodAngle(double angle){
    return Commands.startEnd(() -> setAngle(angle), () -> setAngle(12));
  }

  public boolean readyToShoot(){
    return Math.abs(getAngleDegrees() - setPoint) < HoodConstants.kAngleTolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("ShooterSubsystem/Hood New", hood.getEncoder().getPosition());
    SmartDashboard.putNumber("Hood Angle", getAngleDegrees());
    SmartDashboard.putNumber("Hood setpoint", setPoint);
    SmartDashboard.putBoolean("Hood ready to shoot", readyToShoot());
  }
}
