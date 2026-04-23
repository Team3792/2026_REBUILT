// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Managers;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class GameManager {
    public static boolean error = false;
    public static boolean wonAuto;

    //Global triggers for each shift
    public static Trigger transitionShiftTrigger = new Trigger(() -> GameManager.atTransitionShift());
    public static Trigger shiftOneTrigger = new Trigger(() -> GameManager.atShiftOne());
    public static Trigger shiftTwoTrigger = new Trigger(() -> GameManager.atShiftTwo());
    public static Trigger shiftThreeTrigger = new Trigger(() -> GameManager.atShiftThree());
    public static Trigger shiftFourTrigger = new Trigger(() -> GameManager.atShiftFour());
    public static Trigger endGameTrigger = new Trigger(() -> GameManager.atEndGame());


    public GameManager(){
    }

    private static double getTime(){
        return DriverStation.getMatchTime();
    }

    public boolean setAutoWinner(){
        
        String gameData = DriverStation.getGameSpecificMessage(); 
        if (getAllianceColor() == Alliance.Red) {
            if (gameData.charAt(0) == 'R'){
                wonAuto = true;
            } else {
                wonAuto = false;
            }
        } else {
            if (gameData.charAt(0) == 'B'){
                wonAuto = true;
            } else {
                wonAuto = false;
            }
        }

        return wonAuto;
    }


    public static boolean atTransitionShift(){
        double t = getTime();
        return t > 130;
    }

    public static boolean transitionShift(){
        double t = getTime();
        return t > 130;
    }

    public static boolean atShiftOne(){
        double t = getTime() - 3 ;
        return (t <= 130 && t > 105);
    }

    public static boolean shiftOne(){
        double t = getTime();
        return (t <= 130 && t > 105);
    }
    

    public static boolean atShiftTwo(){
        double t = getTime() -5;
        return (t <= 105 && t > 80);
    }

    public static boolean shiftTwo(){
        double t = getTime();
        return (t <= 105 && t > 80);
    }

     public static boolean atShiftThree(){
        double t = getTime() -5;
        return (t <= 80 && t > 55);
     }

     public static boolean shiftThree(){
        double t = getTime();
        return (t <= 80 && t > 55);
     }

     public static boolean atShiftFour(){
        double t = getTime() -5;
        return (t <= 55 && t > 30);
     }

     public static boolean shiftFour(){
        double t = getTime();
        return (t <= 55 && t > 30);
     }
     
     public static boolean atEndGame(){
        double t = getTime() -5;
        return (t <= 30 && t >= 0);
     }

     public static boolean endGame(){
        double t = getTime();
        return (t <= 30 && t >= 0);
     }

     public static boolean activeShifts(){
        if(wonAuto){
            return transitionShift() || shiftTwo() || shiftFour() || endGame();
        }
        else {
            return transitionShift() || shiftOne() || shiftThree() || endGame();
        }
     }


    public static Alliance getAllianceColor(){
        if(DriverStation.getAlliance().isPresent())
            return DriverStation.getAlliance().get();
        return Alliance.Blue; //Default is blue
    }  

    public static void setAllianceColor(){
    }

    public static boolean flipFiledToRed(){
        if(getAllianceColor().equals(Alliance.Red))
            return true;
        else
            return false;
    }
}

