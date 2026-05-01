// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ShooterSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Subsystems.ShooterSubsystem.ShotCalculator.Shot;
import frc.robot.Subsystems.SwerveSubsystem.Swerve;

/** Add your docs here. */
public class ShootOnTheMoveCalculator {

    static Swerve swerve = new Swerve();

    static double tof;
    static double targetHubDistance;
    static Translation2d targetHubPose;

    public record distanceVsTime(double distanceMeters, double timeOfFlight){};

    private static final distanceVsTime[] kDistanceVsTime = {
        new distanceVsTime(0, 0),
        new distanceVsTime(0, 0),
        new distanceVsTime(0, 0),
        new distanceVsTime(0, 0),
        new distanceVsTime(0, 0),
        new distanceVsTime(0, 0),
        new distanceVsTime(0, 0),
        new distanceVsTime(0, 0),
        new distanceVsTime(0, 0),
        new distanceVsTime(0, 0),
    };

    //return time of flight to actual hub
    public static double tofToHub (double distance){
        tof = 0;
        double calibratedTime = kDistanceVsTime.length;
    
        //calculate time of flight for distance to real hub
        for(int i = 0; i < calibratedTime; i++){
            if(distance > kDistanceVsTime[i].distanceMeters)
              tof = linearInterpolation(distance, kDistanceVsTime[i-1], kDistanceVsTime[i]).timeOfFlight();
        }
        return tof; 
    }

    //return distance of target hub based on field relative robot velocity, distance to actual hub, and tof to actual hub
    private static double targetHubDistance(double velocity, double distance, double tof){

        targetHubDistance = velocity * tofToHub(distance);

        for(int i = 0; i < 5; i++){
            tof = tofToHub(targetHubDistance);
            targetHubDistance = velocity * tof;
        }
        return targetHubDistance;
    }

    //return distaneVsTime record based on distance to actual hub
    public static distanceVsTime getTargetHubDistance(double distance){
        double velocity = -swerve.getFieldRelativeSpeeds()[2];
        return new distanceVsTime(targetHubDistance(velocity,distance, tof), tof);
    }

    //return target heading of robot based on target hub
    public static double getTargetHeading(Pose2d robotPose, double distance){

        double targetHubVelocityX = -swerve.getFieldRelativeSpeeds()[0];
        double targetHubVelocityY = -swerve.getFieldRelativeSpeeds()[1];

        double targetHubPositionX = targetHubVelocityX * tofToHub(distance);
        double targetHubPositionY = targetHubVelocityY * tofToHub(distance);

        targetHubPositionX = targetHubDistance(targetHubVelocityX, distance, tof);
        targetHubPositionY = targetHubDistance(targetHubVelocityX, distance, tof);

        targetHubPose =  new Translation2d (targetHubPositionX, targetHubPositionY);
        double targetHeading = targetHubPose.getAngle().getRadians();
    
        return targetHeading;
    }

    //return shot for moving robot
    public static Shot getDynamicShot(double distance){
        return new Shot(0, 0, 0);//ShotCalculator.getStaticShot(getTargetHubDistance(distance).distanceMeters());
    }

    //returns the interpolated time of flight based on distance and two given time vs distance points
    private static distanceVsTime linearInterpolation(double distance, distanceVsTime distance1, distanceVsTime distance2){
        double weight1 = (distance2.distanceMeters() - distance) / (distance2.distanceMeters() - distance1.distanceMeters());
        double timeOfFlight = distance1.timeOfFlight() * weight1 + distance2.timeOfFlight() * (1 - weight1);
        return new distanceVsTime(distance, timeOfFlight);
    }
}
