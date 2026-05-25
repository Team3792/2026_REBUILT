package frc.robot.Subsystems.ShooterSubsystem.Hood;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.HardwareMap;


public class Hood extends SubsystemBase {
  SparkMax hood = new SparkMax(HardwareMap.hoodID, MotorType.kBrushless);
  SparkClosedLoopController controller;
  double setPoint = HoodConstants.kZeroAngle;
  
  public Hood() {
    hood.configure(HoodConstants.configHood(), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    controller = hood.getClosedLoopController();
    zeroHood();
  }

  /**
   * angle of 0 is a ball exit angle along the horizontal axis
   * angle of 90 is ball exit angle along vertical axis
   * @return hood angle in degrees
   */
  public double getAngleDegrees(){
    return hood.getEncoder().getPosition() / HoodConstants.kHoodRatio / HoodConstants.kMotorRatio * 360.0;
  }

  public void setAngle(double angle) {
    if(angle > 50)
      angle = 50;
    setPoint = angle;
    controller.setSetpoint(angle / 360.0 * HoodConstants.kHoodRatio * HoodConstants.kMotorRatio, ControlType.kPosition);
  }

  public void zeroHood(){
    hood.getEncoder().setPosition(HoodConstants.kZeroAngle / 360.0 * HoodConstants.kHoodRatio * HoodConstants.kMotorRatio);
  }

  public Command setHoodAngle(double angle){
    return Commands.startEnd(() -> setAngle(angle), () -> setAngle(HoodConstants.kZeroAngle));
  }

  public boolean readyToShoot(){
    return Math.abs(getAngleDegrees() - setPoint) < HoodConstants.kAngleTolerance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Angle", getAngleDegrees());
    SmartDashboard.putBoolean("Hood ready to shoot", readyToShoot());
  }
}
