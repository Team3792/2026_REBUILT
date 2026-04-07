// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ShooterSubsystem;

import frc.robot.Managers.FieldManager;
import frc.robot.Subsystems.ShooterSubsystem.Shooter.ShooterConstants;

/** Add your docs here. */
public class ShotCalculator {

    public record Shot(double distanceMeters, double omegaRPS, double hoodAngleDegrees){};


    //TODO: must be sorted by distance
    public static final Shot[] kCalibratedShots = {
        //TODO: add a closer shot
        new Shot(1.726, 52, 24),
        new Shot(2.03, 53.5, 25.5),
        new Shot(2.47, 55, 27),
        new Shot(3.03, 56, 33),
        new Shot(3.43, 57, 34),
        new Shot(4.01, 59.5, 37),
        new Shot(4.5, 60, 40),
        new Shot(5.02, 64, 43),
        new Shot(6.1, 70, 43),
        new Shot(7.2, 75, 43),
    };


    public static final Shot kBumperShot = new Shot(0, 45, 20); //Need to determine
    public static final Shot kFeedShot = new Shot(0, 70, 40);
    public static final Shot kZeroShot = new Shot(0, 0, 0);


    public static Shot getStaticShot(double distance){
        //Linear search until calibrated shots is farther
        int calibratedShots = kCalibratedShots.length;
        for (int i = 0; i < calibratedShots; i++){ //Could restructure into while with exit if
            if(distance < kCalibratedShots[i].distanceMeters()){
                if(i == 0) //Closer than first shot
                    return linearInterpolation(distance, kCalibratedShots[0], kCalibratedShots[1]);
                return linearInterpolation(distance, kCalibratedShots[i-1], kCalibratedShots[i]);
            }
        }

        //farther than final shot, use last two shots 
        return linearInterpolation(distance, kCalibratedShots[calibratedShots - 1], kCalibratedShots[calibratedShots - 2]);
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
