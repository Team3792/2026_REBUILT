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

import frc.robot.FeederSubsystem.Feeder;
import frc.robot.FeederSubsystem.FeederConstants;

import frc.robot.IntakeSubsystem.Intake;
import frc.robot.IntakeSubsystem.IntakeConstants;

public class RobotContainer {

  Feeder feeder = new Feeder();
  FeederConstants feederConstants = new FeederConstants();

  Intake intake = new Intake();
  IntakeConstants intakeConstants = new IntakeConstants();

  CommandPS5Controller driver = new CommandPS5Controller(0);
  CommandPS5Controller operator = new CommandPS5Controller(0);

  public RobotContainer() {
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDriverBindings() {
    driver.R1().whileTrue(intake.runIntake(intakeConstants.intakeVoltage));
    driver.L1().whileTrue(intake.runIntake(-intakeConstants.intakeVoltage));
  }
  private void configureOperatorBindings(){
    operator.square().whileTrue(feeder.runFeeder(feederConstants.kFeedVoltage));
    operator.cross().whileTrue(feeder.runFeeder(-feederConstants.kFeedVoltage)); 
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
