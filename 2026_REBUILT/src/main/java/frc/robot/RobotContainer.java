// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.ClimbSubsytem.Climb;
import frc.robot.ClimbSubsytem.ClimbConstants;
import frc.robot.FeederSubsystem.Feeder;
import frc.robot.FeederSubsystem.FeederConstants;

import frc.robot.IntakeSubsystem.Intake;
import frc.robot.IntakeSubsystem.IntakeConstants;
import frc.robot.SwerveSubsystem.DriveCommand;
import frc.robot.SwerveSubsystem.Swerve;
import frc.robot.ShooterSubsystem.Hood.*;

public class RobotContainer {

  Feeder feeder = new Feeder();
  Intake intake = new Intake();
  Climb climb = new Climb();
  Swerve swerve = new Swerve();
  Hood hood = new Hood();

  CommandPS5Controller driver = new CommandPS5Controller(0);
  CommandPS5Controller operator = new CommandPS5Controller(1);

  public RobotContainer() {
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDriverBindings() {
    driver.R1().onTrue(intake.runToPosition(IntakeConstants.kIntakePosition));
    driver.R1().onFalse(intake.runToPosition(IntakeConstants.kStowedPosition));

    driver.R1().whileTrue(intake.runIntake(IntakeConstants.intakeVoltage));
    driver.L1().whileTrue(intake.runIntake(-IntakeConstants.intakeVoltage));

    swerve.setDefaultCommand(new DriveCommand(swerve, () -> driver.getLeftY(), () -> driver.getLeftX(), () -> driver.getRightX()));
    driver.options().onTrue(Commands.runOnce(() -> swerve.resetHeading(0)));
  }
  
  private void configureOperatorBindings(){
    operator.square().whileTrue(feeder.runFeeder(FeederConstants.kFeedVoltage));
    operator.cross().whileTrue(feeder.runFeeder(-FeederConstants.kFeedVoltage)); 

    operator.povUp().whileTrue(climb.climbUpCommand());
    operator.povDown().whileTrue(climb.climbDownCommand());

    operator.L2().onTrue(hood.setPosition(HoodConstants.kZone1Angle));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
