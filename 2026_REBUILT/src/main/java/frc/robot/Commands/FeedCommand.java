// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.FeederSubsystem.Feeder;
import frc.robot.Subsystems.FeederSubsystem.FeederConstants;
import frc.robot.Subsystems.IntakeSubsystem.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FeedCommand extends Command {
  Intake intake;
  Feeder feeder;
  public FeedCommand(Intake intake, Feeder feeder) {
    this.intake = intake;
    this.feeder = feeder;
    addRequirements(intake, feeder);
  }

  @Override
  public void initialize() {
    feeder.setConveyorVoltage(FeederConstants.kConveyorVoltage);
    feeder.setKickerVoltage(FeederConstants.kKickerVoltage);
  }

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
    feeder.setConveyorVoltage(0);
    feeder.setKickerVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
