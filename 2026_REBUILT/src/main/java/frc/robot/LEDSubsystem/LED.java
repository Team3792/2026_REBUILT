// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LEDSubsystem;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LED extends SubsystemBase {
  private final int kPort = 1;
  private final int kLength = 67; 
  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;

  public LED(){
    led = new AddressableLED(kPort);
    ledBuffer = new AddressableLEDBuffer(kLength);
    led.setLength(kLength);
    led.setData(ledBuffer);
    led.start();                                                                                                                                                                    
  } 
private final LEDPattern coolRainbow = LEDPattern.rainbow(255,128);
// Adust spacing to be how many LEDs per meter
private static final Distance kLedSpacing = edu.wpi.first.units.Units.Meters.of(1.1/67);
private final LEDPattern scrollingRainbowLedPattern =
    coolRainbow.scrollAtAbsoluteSpeed(
      edu.wpi.first.units.Units.MetersPerSecond.of(1), kLedSpacing);
  @Override
  public void periodic() {
    scrollingRainbowLedPattern.applyTo(ledBuffer);
    led.setData(ledBuffer);
    // This method will be called once per scheduler run
  }
public Command runPattern(LEDPattern pattern) {
  return run(() -> pattern.applyTo(ledBuffer));
}
}
