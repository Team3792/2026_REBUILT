package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Managers.FieldManager;
import frc.robot.Subsystems.ShooterSubsystem.ShotCalculator;
import frc.robot.Subsystems.ShooterSubsystem.Hood.*;
import frc.robot.Subsystems.ShooterSubsystem.Shooter.Shooter;
import frc.robot.Subsystems.ShooterSubsystem.Shooter.ShooterConstants;
import frc.robot.Subsystems.ShooterSubsystem.ShotCalculator.Shot;

public class SpinUpCommand extends Command {
  Shooter shooter;
  Hood hood;
  Supplier<Pose2d> shooterPose;
  ShotType shotType;

  private static double shotDistanceOffsetBase = 0; 
  private static double shotDistanceOffsetChange = 0;

  public enum ShotType{
    Hub,
    FixedBumper,
    Feed,
    FixedFeed,
    Automatic
  }
  
  public SpinUpCommand(Hood hood, Shooter shooter, Supplier<Pose2d> shooterPose, ShotType shotType) {
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
    if(shotType == ShotType.FixedBumper){
        hood.setAngle(ShotCalculator.kBumperShot.hoodAngleDegrees());
        shooter.setRPS(ShotCalculator.kBumperShot.omegaRPS());
    } else if(shotType == ShotType.FixedFeed){
        hood.setAngle(ShotCalculator.kFeedShot.hoodAngleDegrees());
        shooter.setRPS(ShotCalculator.kFeedShot.distanceMeters());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    // if(shotType == ShotType.FixedBumper || shotType == ShotType.FixedFeed)
    //   return;

    // ShotType trueShotType = shotType;  
    // if(shotType == ShotType.Automatic){
    //   trueShotType = FieldManager.getShotType(shooterPose.get());
    // }

    // double distanceToTarget = FieldManager.getTargetDistance(shooterPose.get(), trueShotType);
    // Shot targetShot = ShotCalculator.getStaticShot(distanceToTarget + shotDistanceOffsetBase + shotDistanceOffsetChange, trueShotType);
    // hood.setAngle(targetShot.hoodAngleDegrees());
    // shooter.setRPS(targetShot.omegaRPS());
    hood.setAngle(20);
    shooter.setRPS(30);
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
