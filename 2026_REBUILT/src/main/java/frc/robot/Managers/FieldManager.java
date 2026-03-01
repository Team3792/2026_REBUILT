// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Managers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Managers.GameManager.AllianceColor;
import frc.robot.Subsystems.SwerveSubsystem.Swerve;

/** Add your docs here. */
public class FieldManager {



    //coordinates in meters
    //pose based off bottom left of field elements

    //hub
    Translation2d blueHubPos = new Translation2d(0,0);
    Translation2d redHubPos = new Translation2d(0,0);
    
    //trench
    Translation2d bottomRedTrench = new Translation2d(0,0);
    Translation2d topRedTrench = new Translation2d(0,0);
    Translation2d bottomBlueTrench = new Translation2d(0,0);
    Translation2d topBlueTrench = new Translation2d(0,0);

    private double trenchWidth = 1.278636; 
    private double trenchDepth = 1.194;

    private double trenchZoneDepth = 1;
    private double trenchZoneWidth = trenchDepth;





    //bump 

    //climb
    Translation2d blueTowerPos = new Translation2d(0,0);
    Translation2d redTowerPos = new Translation2d(0,0);

    
    // public void setFieldElementPoses(AllianceColor color){
    //     switch (color) {
    //         case Blue:

    // }

    // public boolean trenchZoneSize(){
    //     Swerve.getPose.
    // }

    // //use robot pose to determine if robot is within a trench zone
    // public boolean inTrenchZone(Pose2d robotPose){
    //     return false;
    // }
    
    
}
