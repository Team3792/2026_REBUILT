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
    Timer timer;
    public GameManager(){
    }


    public static boolean atTransitionShift(){
        if (Timer.getMatchTime() > 130){
            return true;
        } else {
            return false;
        }
    }

    public static boolean atShiftOne(){
        if (130 > Timer.getMatchTime() && Timer.getMatchTime() < 105){
            return true;
        } else {
            return false;
        }
    }

    public static boolean atShiftTwo(){
        if (105 > Timer.getMatchTime() && Timer.getMatchTime() < 80){
            return true;
        } else{
            return false;
        }
    }

     public static boolean atShiftThree(){
        if (80 > Timer.getMatchTime() && Timer.getMatchTime() < 55){
            return true;
        } else{
            return false;
        }
    }

     public static boolean atShiftFour(){
        if (55 > Timer.getMatchTime() && Timer.getMatchTime() < 30){
            return true;
        } else{
            return false;
        }
    }

     public static boolean atEndGame(){
        if (30 > Timer.getMatchTime() && Timer.getMatchTime() < 0){
            return true;
        } else{
            return false;
        }
    }

    public static Alliance getAllianceColor(){
        if(alliance.isPresent())
            return alliance.get();
        return Alliance.Blue; //Default is blue
    }  
}

