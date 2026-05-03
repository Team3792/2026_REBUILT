// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.SpinUpCommand.ShotType;
import frc.robot.Managers.FieldManager;
import frc.robot.Managers.GameManager;
import frc.robot.util.JoystickInput;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  /** Creates a new DriveCommand. */
  Swerve swerve;
  Supplier<Double> xSupplier, ySupplier, tSupplier;
  Supplier<Boolean> headingLockSupplier;
  double vx, vy, omega; //stored to access last
  JoystickInput translationJoystick = new JoystickInput(SwerveConstants.kMaxLinearVelocity, 0.05); //used for both x and y
  JoystickInput rotationJoystick = new JoystickInput(SwerveConstants.kMaxOmega, 0.05);
  ProfiledPIDController headingLockController = SwerveConstants.kHeadingLockControllerConfig.getController();
  boolean headingLock = false;


  public DriveCommand(Swerve swerve, Supplier<Double> x, Supplier<Double> y, Supplier<Double> t, Supplier<Boolean> headingLock) {
    this.swerve = swerve;
    this.xSupplier = x;
    this.ySupplier = y;
    this.tSupplier = t;
    this.headingLockSupplier = headingLock;
    headingLockController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    vx = translationJoystick.get(xSupplier.get());
    vy = translationJoystick.get(ySupplier.get());
  
    if(headingLockSupplier.get()){
      //Determine setpoint 
      headingLockController.setGoal(FieldManager.getTargetRotation(swerve.getPose(), ShotType.Automatic).getRadians());
      if(!headingLock){
        headingLockController.reset(swerve.getPoseHeading().getRadians(), omega);
        headingLock = true;
      }
      omega = headingLockController.getSetpoint().velocity + headingLockController.calculate(swerve.getPoseHeading().getRadians());
    } else {
      omega = rotationJoystick.get(tSupplier.get());
      headingLock = false;
    }

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, vy, omega);

    //Field relative transformation
    Rotation2d heading = swerve.getPoseHeading();
    //System.out.println(GameManager.getAllianceColor());
    if(GameManager.getAllianceColor() == Alliance.Red)
      heading = heading.plus(new Rotation2d(Math.PI)); //Rotate heading for red alliance
    //System.out.println(heading.getDegrees());
    
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, heading);
    //TODO: check signs on vx and vy for different alliance colors 

    swerve.drive(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
