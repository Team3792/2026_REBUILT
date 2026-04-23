// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.LEDSubsystem;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * 6328's 2025 code uses addressable LED's so you can look at their code
 * They also have a post in chief delphi 2025 that explains there LED process clearly:
 * https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2025-build-thread/477314/420
 */
public class LED extends SubsystemBase {
  private final int kPort = 0;
  private final int kLength = 65; 
  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;

  public LED(){
    led = new AddressableLED(kPort);
    ledBuffer = new AddressableLEDBuffer(kLength);
    led.setLength(kLength);
    led.setData(ledBuffer);
    led.start();                                                                                                                                                                    
  } 
  // private final LEDPattern coolRainbow = LEDPattern.rainbow(255,128);
  // // Adjust spacing to be how many LEDs per meter
  // private static final Distance kLedSpacing = edu.wpi.first.units.Units.Meters.of(1.1/67);
  
  // // private final LEDPattern scrollingRainbowLedPattern =
  // //   coolRainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

  public void setLEDColor(LEDPattern pattern) {
    pattern.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  public Command runPattern(LEDPattern pattern) {
    return run(() -> setLEDColor(pattern));
  }

  @Override  
  public void periodic() {
  //This method will be called once per scheduler run

  //setLEDColor(LEDConstants.kClimbPattern);
    //led.setData(ledBuffer)
    
  }

}
