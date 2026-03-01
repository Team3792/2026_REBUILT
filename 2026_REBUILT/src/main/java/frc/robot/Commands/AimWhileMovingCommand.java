// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ShooterSubsystem.Shooter.ShooterConstants;
import frc.robot.Subsystems.SwerveSubsystem.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimWhileMovingCommand extends Command {
  /** Creates a new TurnToTargetHubCommand. */
  Swerve swerve;
  Pose2d hubPose;
  ProfiledPIDController headingPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(360, 720));
  private Supplier<Double> xSupplier, ySupplier;

  double targetHubPositionX;
  double targetHubPositionY;

  double newRobotToHubDistance;

  double ballFlightTime;

  public AimWhileMovingCommand(Swerve swerve, Pose2d hubPose, Supplier<Double> x, Supplier<Double> y ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.hubPose = hubPose;
    this.xSupplier = x;
    this.ySupplier = y;

    headingPID.enableContinuousInput(-180, 180);

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeed = xSupplier.get();
    double ySpeed = ySupplier.get();

    Pose2d currentPose = swerve.getPose();
    Translation2d robotToHub = hubPose.getTranslation().minus(currentPose.getTranslation());

    //double robotToHubDistance = robotToHub.getDistance(currentPose.getTranslation());

    double robotToHubDistanceX = robotToHub.getX();
    double robotToHubDistanceY = robotToHub.getY();

    //target hub is where robot should aim; not at the hub istelf, unless robot Velocity = 0
    //target hub is moving in opposite direction of robot
    double targetHubVelocityX = -swerve.getFieldRelativeSpeeds()[0];
    double targetHubVelocityY = -swerve.getFieldRelativeSpeeds()[1];
    double targetHubVelocity = Math.hypot(targetHubVelocityX, targetHubVelocityY);
    
    
    //update target hub position by calculating distance to hub using ball time of flight
    //using distance, recalculate time of flight
    //repeat until error is low enough
    for(int i = 0; i < ShooterConstants.targetHubPoseIterations(targetHubVelocity); i++){
      if(i == 0){
        //calculate based on actual hub pose once
        targetHubPositionY =  targetHubVelocityY * ShooterConstants.fuelFlightTimeVsDistance(robotToHubDistanceY);
        targetHubPositionX = targetHubVelocityX * ShooterConstants.fuelFlightTimeVsDistance(robotToHubDistanceX);
        newRobotToHubDistance = Math.hypot(targetHubPositionX, targetHubPositionY);
        ballFlightTime = ShooterConstants.fuelFlightTimeVsDistance(newRobotToHubDistance);
      }else{
        //calculate target hub pose based on time of flight; iteratively
        targetHubPositionY =  targetHubVelocityY * ballFlightTime;
        targetHubPositionX = targetHubVelocityX * ballFlightTime;
        newRobotToHubDistance = Math.hypot(targetHubPositionX, targetHubPositionY);
        ballFlightTime = ShooterConstants.fuelFlightTimeVsDistance(newRobotToHubDistance);
      }
    }
    
    //angle of target hub
    double targetYaw = Math.atan2(targetHubPositionX, targetHubPositionY);

    double PIDOutput = headingPID.calculate(swerve.getRotation2dHeading().getDegrees(), targetYaw);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, PIDOutput);
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
