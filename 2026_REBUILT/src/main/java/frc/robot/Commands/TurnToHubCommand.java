// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveSubsystem.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToHubCommand extends Command {
  /** Creates a new TurnToHubCommand. */

  Swerve swerve;
  Pose2d hubPose;
  ProfiledPIDController tController = new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(360, 720));
  private Supplier<Double> xSupplier, ySupplier;

  public TurnToHubCommand(Swerve swerve, Pose2d hubPose, Supplier<Double> x, Supplier<Double> y) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.hubPose = hubPose;
    this.xSupplier = x;
    this.ySupplier = y;

    tController.enableContinuousInput(-180, 180);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = swerve.getPose();
    var robotToHub = hubPose.getTranslation().minus(currentPose.getTranslation());
    double targetYaw = robotToHub.getAngle().getDegrees();

    double xSpeed = xSupplier.get();
    double ySpeed = ySupplier.get();
    double tOut = tController.calculate(swerve.getRotation2dHeading().getDegrees(), targetYaw);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, tOut);
    swerve.drive(chassisSpeeds, swerve.getRotation2dHeading());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
