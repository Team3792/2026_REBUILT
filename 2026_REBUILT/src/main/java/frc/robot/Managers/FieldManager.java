// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Managers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class FieldManager {

    //coordinates in meters
    //pose based on bottom left of field element
    /*
    * 
    * -------
    * -------
    * x------
    * 
    */

    //hub
    Translation2d blueHubPos = new Translation2d(0,0);
    Translation2d redHubPos = new Translation2d(0,0);
    
    //trench
    Translation2d bottomRedTrench = new Translation2d(0,0);
    Translation2d topRedTrench = new Translation2d(0,0);
    Translation2d bottomBlueTrench = new Translation2d(0,0);
    Translation2d topBlueTrench = new Translation2d(0,0);

    public static double trenchWidth = 1.278636;
    //private double trenchDepth = 1.194;

    //bump

    //climb
    Translation2d blueTowerPos = new Translation2d(0,0);
    Translation2d redTowerPos = new Translation2d(0,0);

    public Translation2d[] distanceToTrenches(Pose2d robotPose){
        Translation2d robotPos = robotPose.getTranslation();

        Translation2d distanceToBottomRed = bottomRedTrench.minus(robotPos);
        Translation2d distanceToTopRed = topRedTrench.minus(robotPos);
        Translation2d distanceToBottomBlue = bottomRedTrench.minus(robotPos);
        Translation2d distanceToTopBlue = topRedTrench.minus(robotPos);
        
        Translation2d[] distanceToTrenches = {distanceToBottomRed,distanceToTopRed, distanceToBottomBlue, distanceToTopBlue};
        return  distanceToTrenches;

    }

}


