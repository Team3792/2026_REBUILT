// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  /** Creates a new DriveCommand. */
  Swerve swerve;
  private Supplier<Double> xSupplier, ySupplier, tSupplier; 



  public DriveCommand(Swerve swerve, Supplier<Double> x, Supplier<Double> y, Supplier<Double> t) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.xSupplier = x;
    this.ySupplier = y;
    this.tSupplier = t;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = 3 * xSupplier.get();
    double ySpeed = 3 * ySupplier.get();
    double tSpeed = 5 * tSupplier.get();

    //apply deadband
    xSpeed = Math.abs(xSpeed) < 0.05 ? 0.0 : xSpeed;
    ySpeed = Math.abs(ySpeed) < 0.05 ? 0.0 : ySpeed;
    tSpeed = Math.abs(tSpeed) < 0.05 ? 0.0 : tSpeed;


    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, tSpeed);

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
