// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Managers.FieldManager;
import frc.robot.Subsystems.ShooterSubsystem.ShotCalculator;
import frc.robot.Subsystems.ShooterSubsystem.Hood.*;
import frc.robot.Subsystems.ShooterSubsystem.Shooter.Shooter;
import frc.robot.Subsystems.ShooterSubsystem.ShotCalculator.Shot;




/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */

  Shooter shooter;
  Hood hood;
  Supplier<Pose2d> shooterPose;
  
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
  public void execute() {                                                       //fudge factor 
    double robotToHubDistance = FieldManager.getRobotToHubDis(shooterPose.get()) - 0.20;//need to update
    Shot targetShot = ShotCalculator.getStaticShot(robotToHubDistance);
    //TODO: implmenet increments in respective classes
    hood.setAngle(targetShot.hoodAngleDegrees());
    shooter.setRPS(targetShot.omegaRPS());
  
    // hood.setAngle(targetHoodAngle + hood.getHoodIncrimet());
    // shooter.setRPS(targetRPS + shooter.getRPSIncriment());
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.spinDown();
    hood.setAngle(12);
  }

  public static Command fixedShot(Shot shot, Hood hood, Shooter shooter){
    return Commands.startEnd(
      () -> {
        hood.setAngle(shot.hoodAngleDegrees());
        shooter.setRPS(shot.omegaRPS());
      },
      () -> {
        hood.setAngle(12);
        shooter.spinDown();;
      },
      shooter, hood);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
