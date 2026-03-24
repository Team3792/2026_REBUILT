// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Managers;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class GameManager {
    public static Optional<Alliance> alliance = DriverStation.getAlliance();
    public GameManager(){
    }

    private static double getTime(){
        return DriverStation.getMatchTime();
    }

    public static boolean atTransitionShift(){
        double t = getTime();
        return t > 130;
    }

    public static boolean atShiftOne(){
        double t = getTime();
        return (t <= 130 && t > 105);
    }

    public static boolean atShiftTwo(){
        double t = getTime();
        return (t <= 105 && t > 80);
    }

     public static boolean atShiftThree(){
        double t = getTime();
        return (t <= 80 && t > 55);
     }

     public static boolean atShiftFour(){
        double t = getTime();
        return (t <= 55 && t > 30);
     }
     
     public static boolean atEndGame(){
        double t = getTime();
        return (t <= 30 && t >= 0);
     }

    public static Alliance getAllianceColor(){
        if(alliance.isPresent())
            return alliance.get();
        return Alliance.Blue; //Default is blue
    }  
}

