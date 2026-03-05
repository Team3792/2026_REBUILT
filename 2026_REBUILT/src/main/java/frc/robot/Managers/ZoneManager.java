// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Managers;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class ZoneManager {
    double robotX = 0;
    double robotY = 0;
    double topTrenchWidth = 0;
    double topTrenchHeight = 0;


    public trenchZone getRobotZone(){
        if(0<robotX && topTrenchWidth>robotX && 0<robotY && topTrenchHeight>robotY){
        return trenchZone.topTrench;
        }
    }


    public enum trenchZone{
        topTrench, bottomTrench
    }


    


}
