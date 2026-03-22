// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Managers;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class GameManager {
    public static Optional<Alliance> alliance = DriverStation.getAlliance();
    public GameManager(){}

    public static Alliance getAllianceColor(){
        if(alliance.isPresent())
            return alliance.get();
        return Alliance.Blue; //Default is blue
    }  
}

