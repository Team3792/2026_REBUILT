// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LEDSubsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private final int ledPort = 0;
  private final int ledLength = 1; 
  AddressableLED led;
  AddressableLEDBuffer ledBuffer;
  public LED(){
    led = new AddressableLED(ledPort);

  } 

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
