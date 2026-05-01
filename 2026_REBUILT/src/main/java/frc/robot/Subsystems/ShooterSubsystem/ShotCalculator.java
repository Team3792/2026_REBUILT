// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ShooterSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Commands.SpinUpCommand.ShotType;
import frc.robot.Managers.GameManager;

/** Add your docs here. */
public class ShotCalculator {

    public record Shot(double distanceMeters, double omegaRPS, double hoodAngleDegrees){};

    //TODO: must be sorted by distance
    public static final Shot[] kCalibratedShots = {
        //TODO: add a closer shot
        // new Shot(1.726, 52, 24),
        // new Shot(2.03, 53.5, 25.5),
        // new Shot(2.47, 55, 27),
        // new Shot(3.03, 56, 33),
        // new Shot(3.43, 57, 34),
        // new Shot(4.01, 59.5, 37),
        // new Shot(4.5, 60, 40),
        // new Shot(5.02, 64, 43),
        // new Shot(6.1, 70, 43),
        // new Shot(7.2, 75, 43),

        new Shot(1.52, 44, 20),
        new Shot(2.14, 48.6, 20),
        new Shot(2.54,50,24),
        new Shot(3.0, 52,28),
        new Shot(3.5, 57,28),
        new Shot(4, 56,32),
        new Shot(4.5,60, 32),
        new Shot(5.67,70,43),
        new Shot(6.77,75,43),

    };

    public static final Shot[] kCalibratedFeedShots = {
        new Shot(4, 50, 45),
        new Shot(8, 70, 45)
    };

    public static final Shot kBumperShot = new Shot(0, 43.5,16); //Need to determine
    public static final Shot kFeedShot = new Shot(0, 70, 45);
    public static final Shot kZeroShot = new Shot(0, 0, 0);
    //used on controller 3
    public static final Shot kTestShot = new Shot(0,40,20);

  

    public static Shot getStaticShot(double distance, ShotType shotType){
        //Linear search until calibrated shots is farther
        Shot[] lookupTable;
        if(shotType == ShotType.Hub)
            lookupTable = kCalibratedShots;
        else
            lookupTable = kCalibratedFeedShots;
        int calibratedShots = lookupTable.length;
        for (int i = 0; i < calibratedShots; i++){ //Could restructure into while with exit if
            if(distance < lookupTable[i].distanceMeters()){
                if(i == 0) //Closer than first shot
                    return linearInterpolation(distance, lookupTable[0], lookupTable[1]);
                return linearInterpolation(distance, lookupTable[i-1], lookupTable[i]);
            }
        }

        //farther than final shot, use last two shots 
        return linearInterpolation(distance, lookupTable[calibratedShots - 1], lookupTable[calibratedShots - 2]);
    }

    public static Shot[] getLookupTable(Pose2d shooterPose, ShotType shotType){
        if (GameManager.getAllianceColor() == Alliance.Blue){
            if (shotType == ShotType.Feed || shooterPose.getX() > 5.0)
                return kCalibratedFeedShots;
            return kCalibratedShots;
        } else{
            if(shotType == ShotType.Feed || shooterPose.getX() < 11.0)
                return kCalibratedFeedShots;
            return kCalibratedShots;
        }
    }


    //Returns the intorpolated shot based on distance and two given shots 
    //Distance need not be between shot1 and shot2, which is the case on the endpoints
    private static Shot linearInterpolation(double distance, Shot shot1, Shot shot2){
        //weight1 = measure of how close to shot1 it is (1 means it is shot 1, 0 means it should be shot 2)
        double weight1 = (shot2.distanceMeters() - distance) / (shot2.distanceMeters() - shot1.distanceMeters());
        double targetRPS = shot1.omegaRPS() * weight1 + shot2.omegaRPS() * (1 - weight1);
        double targetAngle = shot1.hoodAngleDegrees() * weight1 + shot2.hoodAngleDegrees() * (1 - weight1);
          return new Shot(distance, targetRPS, targetAngle);
    }
}
