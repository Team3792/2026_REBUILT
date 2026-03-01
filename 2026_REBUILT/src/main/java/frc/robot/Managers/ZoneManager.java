// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Managers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.spline.PoseWithCurvature;
import frc.robot.Managers.FieldManager;
import frc.robot.Subsystems.ShooterSubsystem.Hood.Hood;
import frc.robot.Subsystems.ShooterSubsystem.Hood.HoodConstants;

/** Add your docs here. */
public class ZoneManager {

    FieldManager fieldManager = new FieldManager();

    Hood hood = new Hood();
    HoodConstants hoodConstants = new HoodConstants();

    //units: meters

//trench zone;
    
    //trench
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
        return robotVelocity * hoodConstants.timeToStow(robotVelocity);
    }

    // public double[] trenchZoneCoordinates(double robotVelocity){
    //     double trenchZoneX = fieldManager.trenchWidth - trenchZoneSize(robotVelocity);
    //     double trenchZoneY = trenchZoneSize(robotVelocity);
    //     double[] trenchZone = {trenchZoneX, trenchZoneY};
    //     return trenchZone;
    // }

    // public boolean inTrenchZone(Pose2d robotPose){

    //     double robotX = robotPose.getX();
    //     double robotY = robotPose.getY();

    //     if(robotY - )

    // }

//climb zones; can't shoot while behind shooter

}
