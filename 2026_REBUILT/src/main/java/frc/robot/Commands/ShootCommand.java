// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.FeederSubsystem.*;
import frc.robot.Subsystems.LEDSubsystem.*;
import frc.robot.Subsystems.ShooterSubsystem.Hood.*;
import frc.robot.Subsystems.ShooterSubsystem.Shooter.Shooter;
import frc.robot.Subsystems.ShooterSubsystem.Shooter.ShooterConstants;
import frc.robot.Subsystems.SwerveSubsystem.Swerve;




/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */

  Shooter shooter = new Shooter();
  Hood hood = new Hood();
  Swerve swerve;
  LED led = new LED();
  Feeder feeder = new Feeder();


  double targetRPM;
  double rangePercentage;
  double targetHoodAngle;

  PIDController shooterPID = new PIDController(ShooterConstants.kP,ShooterConstants.kI,ShooterConstants.kD);
  PIDController hoodPID = new PIDController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD);

  Pose2d hubPose;
 
  public ShootCommand(Swerve swerve, Pose2d hubPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.hubPose = hubPose;
    addRequirements(shooter);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.setLEDColor(LEDConstants.kShootingPattern);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d currentPos = swerve.getPose();
    Translation2d robotToHubPose = hubPose.getTranslation().minus(currentPos.getTranslation());
    //double robotToHubDistance = Math.hypot((robotToHubPose.getX(),robotToHubPose.getY());
    //velocity of robot relative to field; in x direction
    double fieldRelativeVelocityX = swerve.getFieldRelativeSpeeds()[0];
    double filedRelativeVelocityY = swerve.getFieldRelativeSpeeds()[1];

    double robotToHubDistance = robotToHubPose.getDistance(currentPos.getTranslation());




    for(int i = 1; i < ShooterConstants.distanceVSRPM[0].length; i++){

      if(robotToHubDistance > ShooterConstants.distanceVSRPM[0][i]){
        //return target flywheel RPM
        targetRPM = ShooterConstants.distanceVSRPM[1][i-1];

        //returns what percentage of the distance range the robot is in
        double lowerDistance = ShooterConstants.distanceVsHoodAngle[0][i-1];
        double upperDistance = ShooterConstants.distanceVsHoodAngle[0][i];
        rangePercentage = (robotToHubDistance - lowerDistance) / (upperDistance - lowerDistance);

        //interpolate hood angle based on rangePercentage
        double lowerHoodAngle = ShooterConstants.distanceVsHoodAngle[1][i-1];
        double upperHoodAngle = ShooterConstants.distanceVsHoodAngle[1][i];
        targetHoodAngle = ((upperHoodAngle-lowerHoodAngle) * rangePercentage) + lowerHoodAngle;
        break;
      }
      
    }
    
  
    //based off best fit curve
    //double hoodPower = hoodPID.calculate(hood.hoodAngle(), HoodConstants.hoodAngleBestFit) + HoodConstants.kFrictionFF;
    //based off array value interpolation
    //double hoodPower = hoodPID.calculate(hood.hoodAngle(), targetHoodAngle) + HoodConstants.kFrictionFF;
    //hood.setVoltage(hoodPower);

    //TODO: add velocity feedforward (could do in slot0 configs)
    double shooterPower = shooterPID.calculate(shooter.getRPM(), targetRPM) + ShooterConstants.kFrictionFF;
    shooter.setVoltage(shooterPower);

    feeder.setSubsystemVoltage();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //sets rpm of shooter to some constants rpm when not shooting; talkes less time to spin up
    //shooter.setVoltage(shooterPID.calculate(shooter.getRPM(), shooterConstants.coastRPM));
    led.setLEDColor(LEDConstants.kIdlePattern);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
