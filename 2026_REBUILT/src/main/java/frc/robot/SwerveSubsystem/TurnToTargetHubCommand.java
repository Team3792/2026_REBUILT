// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterSubsystem.Shooter.ShooterConstants;
import frc.robot.SwerveSubsystem.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToTargetHubCommand extends Command {
  /** Creates a new TurnToTargetHubCommand. */
  Swerve swerve;
  Pose2d hubPose;
  ProfiledPIDController anglePID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(360, 720));
  private Supplier<Double> xSupplier, ySupplier;

  public TurnToTargetHubCommand(Swerve swerve, Pose2d hubPose, Supplier<Double> x, Supplier<Double> y ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.hubPose = hubPose;
    this.xSupplier = x;
    this.ySupplier = y;

    anglePID.enableContinuousInput(-180, 180);

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = swerve.getPose();
    Translation2d robotToHub = hubPose.getTranslation().minus(currentPose.getTranslation());
    double robotToHubDistance = robotToHub.getDistance(currentPose.getTranslation());
    double robotToHubDistanceX = robotToHub.getX();
    double robotToHubDistanceY = robotToHub.getY();
    //target hub is where robot should aim; not at the hub istelfs, unless robot V_y = 0
    //velocity_y of target hub = opposite of robot velocity_y: m/s
    double targetHubVelocityY = -swerve.getFieldRelativeSpeeds()[1];
    double targetHubPositionY =  targetHubVelocityY * ShooterConstants.fuelFlightTimeVsDistance(robotToHubDistance);
    double robotToTargetHubY = hubPose.getTranslation().getY() - targetHubPositionY;

    double targetYaw = Math.atan2(robotToHubDistanceX, robotToTargetHubY);

    double xSpeed = xSupplier.get();
    double ySpeed = ySupplier.get();
    double tOut = anglePID.calculate(swerve.getRotation2dHeading().getDegrees(), targetYaw);

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
