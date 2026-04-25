// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Managers.FieldManager;
import frc.robot.Subsystems.ShooterSubsystem.ShootOnTheMoveCalculator;
import frc.robot.Subsystems.ShooterSubsystem.ShotCalculator.Shot;
import frc.robot.Subsystems.ShooterSubsystem.Hood.Hood;
import frc.robot.Subsystems.ShooterSubsystem.Shooter.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootOnTheMove extends Command {

  Shooter shooter;
  Hood hood;
  Supplier<Pose2d> shooterPose;

  /** Creates a new ShootOnTheMove. */
  public ShootOnTheMove(Hood hood, Shooter shooter, Supplier<Pose2d> shooterPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hood = hood;
    this.shooter = shooter;
    this.shooterPose = shooterPose;
    addRequirements(shooter, hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     double robotToHubDistance = FieldManager.getRobotToHubDis(shooterPose.get()) - 0.20;
    Shot targetShot = ShootOnTheMoveCalculator.getDynamicShot(robotToHubDistance);
    hood.setAngle(targetShot.hoodAngleDegrees());
    shooter.setRPS(targetShot.omegaRPS());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.spinDown();
    hood.setAngle(12);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
