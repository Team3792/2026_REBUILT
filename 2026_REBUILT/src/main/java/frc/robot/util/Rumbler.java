// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Managers.GameManager;

public class Rumbler {
  // intensity on a scale of 0-1 and duration in seconds
  public static Command rumble(double intensity, double duration, CommandXboxController controller){
      return Commands.startEnd(() -> controller.setRumble(RumbleType.kBothRumble, intensity), () -> controller.setRumble(RumbleType.kBothRumble, 0)).withTimeout(duration);
  }

  public static Command rumble(double intensity, CommandXboxController controller){
    return Commands.startEnd(() -> controller.setRumble(RumbleType.kBothRumble, intensity), () -> controller.setRumble(RumbleType.kBothRumble, 0));
  }

  public static void bindTransitionRumbles(CommandXboxController controller){
    GameManager.transitionShiftTrigger.onTrue(rumble(1, 4, controller));
    GameManager.shiftOneTrigger.onTrue(rumble(1, 4, controller));
    GameManager.shiftTwoTrigger.onTrue(rumble(1, 4, controller));
    GameManager.shiftThreeTrigger.onTrue(rumble(1, 4, controller));
    GameManager.shiftFourTrigger.onTrue(rumble(1, 4, controller));
    GameManager.endGameTrigger.onTrue(rumble(1, 4, controller));
  }
}
