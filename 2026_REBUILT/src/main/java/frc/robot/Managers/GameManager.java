// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Managers;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class GameManager {
    public Optional<Alliance> alliance = DriverStation.getAlliance();
    public GameManager(){}


    public AllianceColor getAllianceColor(){
     if (alliance.get() == Alliance.Blue){
        return AllianceColor.Blue;
     } else {
        return AllianceColor.Red;
     }   
    }

    public enum AllianceColor{
        Blue, Red
    }
    
    // public double gameState(){
        
    // }

}

