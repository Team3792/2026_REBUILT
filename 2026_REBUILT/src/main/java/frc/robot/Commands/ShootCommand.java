// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Managers.FieldManager;
import frc.robot.Subsystems.FeederSubsystem.*;
import frc.robot.Subsystems.LEDSubsystem.*;
import frc.robot.Subsystems.ShooterSubsystem.Hood.*;
import frc.robot.Subsystems.ShooterSubsystem.Shooter.Shooter;
import frc.robot.Subsystems.ShooterSubsystem.Shooter.ShooterConstants;
import frc.robot.Subsystems.SwerveSubsystem.Swerve;




/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */

  Shooter shooter;
  Hood hood;
  Supplier<Pose2d> shooterPose;


  double targetRPS;
  double rangePercentage;
  double targetHoodAngle;

  PIDController shooterPID = new PIDController(ShooterConstants.kP,ShooterConstants.kI,ShooterConstants.kD);
  PIDController hoodPID = new PIDController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD);

  Pose2d hubPose;
 
  public ShootCommand(Hood hood, Shooter shooter, Supplier<Pose2d> shooterPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hood = hood;
    this.shooter = shooter;
    this.shooterPose = shooterPose;
    addRequirements(shooter, hood);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // led.setLEDColor(LEDConstants.kShootingPattern);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Translation2d robotToHubPose = hubPose.getTranslation().minus(currentPos.getTranslation());
    // //double robotToHubDistance = Math.hypot((robotToHubPose.getX(),robotToHubPose.getY());
    // //velocity of robot relative to field; in x direction
    // double fieldRelativeVelocityX = swerve.getFieldRelativeSpeeds()[0];
    // double filedRelativeVelocityY = swerve.getFieldRelativeSpeeds()[1];

    double robotToHubDistance = FieldManager.getRobotToHubDis(shooterPose.get());//robotToHubPose.getDistance(currentPos.getTranslation());
    
    for (int i = 0; i < ShooterConstants.kCalibratedShots.length - 1; i++){
    double calibratedDistance = ShooterConstants.kCalibratedShots[i].distanceMeters();
    if (robotToHubDistance < calibratedDistance){
      //Linear interpolation between two 
      if(i == 0){
        targetRPS = ShooterConstants.kCalibratedShots[0].omegaRPS();
        targetHoodAngle = ShooterConstants.kCalibratedShots[0].hoodAngleDegrees();
      }
      System.out.println(calibratedDistance);
      double t = (calibratedDistance - robotToHubDistance) / (calibratedDistance - ShooterConstants.kCalibratedShots[i-1].distanceMeters());
      targetRPS = ShooterConstants.kCalibratedShots[i].omegaRPS() * (1 - t)
                 + ShooterConstants.kCalibratedShots[i-1].omegaRPS() *(t); 
      targetHoodAngle = ShooterConstants.kCalibratedShots[i].hoodAngleDegrees() * (1 - t)
                  + ShooterConstants.kCalibratedShots[i-1].hoodAngleDegrees() * t;
      break;
    }
  }


    // for(int i = 1; i < ShooterConstants.distanceVSRPS[0].length; i++){

    //   if(robotToHubDistance > ShooterConstants.distanceVSRPS[0][i]){
    //     //return target flywheel RPM
    //     targetRPS = ShooterConstants.distanceVSRPS[1][i-1];

    //     //returns what percentage of the distance range the robot is in
    //     double lowerDistance = ShooterConstants.distanceVsHoodAngle[0][i-1];
    //     double upperDistance = ShooterConstants.distanceVsHoodAngle[0][i];
    //     rangePercentage = (robotToHubDistance - lowerDistance) / (upperDistance - lowerDistance);

    //     //interpolate hood angle based on rangePercentage
    //     double lowerHoodAngle = ShooterConstants.distanceVsHoodAngle[1][i-1];
    //     double upperHoodAngle = ShooterConstants.distanceVsHoodAngle[1][i];
    //     targetHoodAngle = ((upperHoodAngle-lowerHoodAngle) * rangePercentage) + lowerHoodAngle;
    //     break;
    //   }
    //}

    hood.setAngle(targetHoodAngle);
    shooter.setRPS(targetRPS);
    //shooter.setRPS(shooter.getTargetRPS(robotToHubDistance));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //sets rpm of shooter to some constants rpm when not shooting; talkes less time to spin up
    //shooter.setVoltage(shooterPID.calculate(shooter.getRPM(), shooterConstants.coastRPM));
    shooter.setVoltage(0);
    hood.setAngle(12);
    //led.setLEDColor(LEDConstants.kIdlePattern);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
