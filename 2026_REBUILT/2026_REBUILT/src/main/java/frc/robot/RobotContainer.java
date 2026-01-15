// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class RobotContainer {
  TalonFX motor = new TalonFX(0);
  CommandPS5Controller controller = new CommandPS5Controller(0);

  public void setVoltage(double voltage) {
    motor.setNeutralMode(NeutralModeValue.Coast);
    motor.setVoltage(voltage);
  }
  
  public Command runMotor(double voltage) {
    return Commands.runOnce(() -> setVoltage(voltage));
  }

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.circle().onTrue(runMotor(0.5));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
