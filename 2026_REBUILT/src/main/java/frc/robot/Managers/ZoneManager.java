// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Managers;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Subsystems.ShooterSubsystem.Hood.Hood;
import frc.robot.Subsystems.ShooterSubsystem.Hood.HoodConstants;

/** Add your docs here. */
public class ZoneManager {

    FieldManager fieldManager = new FieldManager();
    
    
    HoodConstants hoodConstants = new HoodConstants();

    double robotX = 0;
    double robotY = 0;
    

    public enum trench{
        bottomRedTrench, // 1
        topRedTrench,    // 2
        bottomBlueTrench,// 3
        topBlueTrench    // 4
    }

    // public double[] distanceToTrenches(Pose2d robotPose){
    
    //     switch(trench){
    //         case bottomRedTrench:


    //     }
    // }

     //radius of zone circle; size based on robotVelocity, and time taken to stow hood
    public double trenchZoneSize(double robotVelocity){
        return robotVelocity * HoodConstants.timeToStow(robotVelocity);
    }

    public double[] trenchZoneCoordinates(double robotVelocity){
        double trenchZoneX = FieldManager.trenchWidth - trenchZoneSize(robotVelocity);
        double trenchZoneY = trenchZoneSize(robotVelocity);
        double[] trenchZone = {trenchZoneX, trenchZoneY};
        return trenchZone;
    }

    // public trenchZone getRobotZone(){
    //     if(0<robotX && topTrenchWidth>robotX && 0<robotY && topTrenchHeight>robotY){
    //     return trenchZone.topTrench;
    //     }
    // }

    // Implement
    public boolean isCloseToTrench(){
        return true;
    }


    // public enum trenchZone{
    //     topTrench, bottomTrench
    // }




    


}
