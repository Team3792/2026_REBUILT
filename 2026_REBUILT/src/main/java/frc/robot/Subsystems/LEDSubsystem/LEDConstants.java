
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.LEDSubsystem;

import java.util.regex.Pattern;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class LEDConstants {
    //ADD LED Patterns here
    public static final LEDPattern kClimbPattern = LEDPattern.solid(Color.kSkyBlue);
    public static final LEDPattern kIntakePattern = LEDPattern.solid(Color.kGold);
    public static final LEDPattern kShootingPattern = LEDPattern.solid(Color.kOrangeRed);
    public static final LEDPattern kIdlePattern = LEDPattern.solid(Color.kSeaGreen);
}
