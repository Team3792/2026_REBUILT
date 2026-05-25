package frc.robot.Subsystems.ShooterSubsystem.Shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.HardwareMap;

public class Shooter extends SubsystemBase {
  Timer timer = new Timer();
  Timer time = new Timer();
  boolean ballShot;
  public static double ballsShot = 0;

  TalonFX rightMotor = new TalonFX(HardwareMap.shootLeaderID.id());
  TalonFX leftMotor = new TalonFX(HardwareMap.shootFollowerID.id());

  MotionMagicVelocityVoltage mmVelocity = new MotionMagicVelocityVoltage(0);
  double targetRPS = 0;

  public Shooter() {
    rightMotor.getConfigurator().apply(ShooterConstants.applyRightMotorConfig());
    leftMotor.getConfigurator().apply(ShooterConstants.applyLeftMotorConfig());

    //right motor follows left motor
    rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    mmVelocity.Slot = 0;
}

public void setVoltage(double voltage){
  VoltageOut output = new VoltageOut(voltage).withEnableFOC(true);
  leftMotor.setControl(output);
}

public Command applyVoltage(double voltage){
  return Commands.runOnce(() -> setVoltage(voltage));
}

public double getRPS(){
  return leftMotor.getVelocity().getValueAsDouble();
}

public void setRPS(double targetVelocity){
  if(targetVelocity > 80)
    targetVelocity = 80;
  targetRPS = targetVelocity;
  leftMotor.setControl(mmVelocity.withVelocity(targetVelocity)); 
}

public void spinDown(){
  targetRPS = 0;
  setVoltage(0);
}

public Command setFlywheelRPS(double targetRPM){
  return Commands.startEnd(() -> setRPS(targetRPM), () -> setRPS(0));
}

public boolean readyToShoot(){
  return Math.abs(getRPS() - targetRPS) < ShooterConstants.kRPSTolerance;
}

@Override
public void periodic() {
    SmartDashboard.putNumber("Shooter RPS", getRPS());
    SmartDashboard.putNumber("Target RPS", targetRPS);
    SmartDashboard.putBoolean("Shooter ready to shoot", readyToShoot());
  }
}
