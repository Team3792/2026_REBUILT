// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ShooterSubsystem.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveSubsystem.Swerve;
import frc.robot.ShooterSubsystem.Shooter.ShooterConstants;
import frc.robot.ShooterSubsystem.Shooter.Shooter;
import frc.robot.ShooterSubsystem.Hood.*;
import frc.robot.LEDSubsystem.*;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */

  Shooter shooter = new Shooter();
  Hood hood = new Hood();
  Swerve swerve;
  LED led = new LED();

  double targetRPM;
  double distanceRange;
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
    var robotToHubPose = hubPose.getTranslation().minus(currentPos.getTranslation());
    double robotToHubDistance = Math.sqrt(Math.pow(robotToHubPose.getX(), 2) + Math.pow(robotToHubPose.getY(), 2));

    for(int i = 0; i < ShooterConstants.distanceVSRPM[0].length; i++){
      if(robotToHubDistance > ShooterConstants.distanceVSRPM[0][i]){
        int index = i-1 ;
        targetRPM = ShooterConstants.distanceVSRPM[1][index];
        //return what percentage of what the distance is of the upper distance value in the rpm range of the 2d array
        distanceRange = (robotToHubDistance - ShooterConstants.distanceVSRPM[1][index -=1]) / 24;
        //multiply upper hood value for given distance range; multiply by percentage calculated above
        targetHoodAngle = ShooterConstants.distanceVsHoodAngle[0][i] * distanceRange;
        break;
      }
      else{
        continue;
      }
    }
    
  

    double hoodPower = hoodPID.calculate(hood.hoodAngle(), HoodConstants.hoodAngleBestFit) + HoodConstants.kFrictionFF;
    //double hoodPower = hoodPID.calculate(hood.hoodAngle(), targetHoodAngle) + HoodConstants.kFrictionFF;
    hood.setVoltage(hoodPower);

    double shooterPower = shooterPID.calculate(shooter.getRPM(), targetRPM) + ShooterConstants.kFrictionFF;
    shooter.setVoltage(shooterPower);
    
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
