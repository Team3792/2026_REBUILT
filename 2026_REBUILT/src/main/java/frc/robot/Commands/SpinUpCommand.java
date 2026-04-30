// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Managers.FieldManager;
import frc.robot.Subsystems.ShooterSubsystem.ShotCalculator;
import frc.robot.Subsystems.ShooterSubsystem.Hood.*;
import frc.robot.Subsystems.ShooterSubsystem.Shooter.Shooter;
import frc.robot.Subsystems.ShooterSubsystem.ShotCalculator.Shot;




/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinUpCommand extends Command {
  /** Creates a new ShootCommand. */

  Shooter shooter;
  Hood hood;
  Supplier<Pose2d> shooterPose;
  ShotType shotType;

  private static double shotDistanceOffsetBase = 0; //- 0.20 - Units.inchesToMeters(22) - 0.4;
  private static double shotDistanceOffsetChange = 0;

  public enum ShotType{
    Hub,
    FixedBumper,
    Feed,
    Automatic
  }
  
  public SpinUpCommand(Hood hood, Shooter shooter, Supplier<Pose2d> shooterPose, ShotType shotType) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hood = hood;
    this.shooter = shooter;
    this.shooterPose = shooterPose;
    this.shotType = shotType;

    SmartDashboard.putNumber("Shot Distance Offset", shotDistanceOffsetChange);

    addRequirements(shooter, hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // led.setLEDColor(LEDConstants.kShootingPattern);
    if(shotType == ShotType.FixedBumper){
        hood.setAngle(ShotCalculator.kBumperShot.hoodAngleDegrees());
        shooter.setRPS(ShotCalculator.kBumperShot.omegaRPS());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    if(shotType == ShotType.FixedBumper) //Skip if bumper shot
      return;

    double distanceToTarget = FieldManager.getTargetDistance(shooterPose.get(), shotType);
    Shot targetShot = ShotCalculator.getStaticShot(distanceToTarget + shotDistanceOffsetBase + shotDistanceOffsetChange);
    hood.setAngle(targetShot.hoodAngleDegrees());
    shooter.setRPS(targetShot.omegaRPS());
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.spinDown();
    hood.setAngle(HoodConstants.kMinAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  public static void incrementOffset(double increase){
    shotDistanceOffsetChange += increase;
    SmartDashboard.putNumber("Shot Distance Offset", shotDistanceOffsetChange);
  }

  public static void resetOffset(){
    shotDistanceOffsetChange = 0;
    SmartDashboard.putNumber("Shot Distance Offset", shotDistanceOffsetChange);
  }
}
