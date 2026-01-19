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

public class RobotContainer {

  Feeder feeder = new Feeder();
  FeederConstants feederConstants = new FeederConstants();

  CommandPS5Controller driver = new CommandPS5Controller(0);
  CommandPS5Controller operator = new CommandPS5Controller(0);

  public RobotContainer() {
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDriverBindings() {
    
  }
  private void configureOperatorBindings(){
    operator.square().whileTrue(feeder.runFeeder(feederConstants.kFeedVoltage));
    operator.cross().whileTrue(feeder.runFeeder(-feederConstants.kFeedVoltage)); 
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
