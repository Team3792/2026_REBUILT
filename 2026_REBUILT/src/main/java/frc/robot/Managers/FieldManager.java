package frc.robot.Managers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Commands.SpinUpCommand.ShotType;

public class FieldManager {
    /**
     * coordinates in meters
     * pose based on bottom left of field element
     */

    //hub
    public static Pose2d blueHubPos = new Pose2d(new Translation2d(4.626,4.035), new Rotation2d(0));
    public static Pose2d redHubPos = new Pose2d(new Translation2d(11.915, 4.035), new Rotation2d(0));

    //feed poses
    public static Pose2d blueFeedTopPos = new Pose2d(new Translation2d(2.5,6), new Rotation2d(0));
    public static Pose2d blueFeedBottomPos = new Pose2d(new Translation2d(2.5, 2), new Rotation2d(0));    
    public static Pose2d redFeedTopPos = new Pose2d(new Translation2d(14,6), new Rotation2d(0));
    public static Pose2d redFeedBottomPos = new Pose2d(new Translation2d(14, 2), new Rotation2d(0));

    //middle line threshold for determining feed pose
    public static double yMiddleThreshold = 4.0;

    //trench
    Translation2d bottomRedTrench = new Translation2d(0,0);
    Translation2d topRedTrench = new Translation2d(0,0);
    Translation2d bottomBlueTrench = new Translation2d(0,0);
    Translation2d topBlueTrench = new Translation2d(0,0);
    public static double trenchWidth = 1.278636;

    public Translation2d[] distanceToTrenches(Pose2d robotPose){
        Translation2d robotPos = robotPose.getTranslation();

        Translation2d distanceToBottomRed = bottomRedTrench.minus(robotPos);
        Translation2d distanceToTopRed = topRedTrench.minus(robotPos);
        Translation2d distanceToBottomBlue = bottomRedTrench.minus(robotPos);
        Translation2d distanceToTopBlue = topRedTrench.minus(robotPos);
        
        Translation2d[] distanceToTrenches = {distanceToBottomRed,distanceToTopRed, distanceToBottomBlue, distanceToTopBlue};
        return  distanceToTrenches;
    }

    public static Pose2d getTargetPose(Pose2d shooterPose, ShotType shotType){
        if(shotType == ShotType.Automatic)
            shotType = FieldManager.getShotType(shooterPose);
        if(shotType == ShotType.Feed)
            return getFeedPose(shooterPose);
        else    
            return getHubPose();
    }

    public static ShotType getShotType(Pose2d shooterPose){
        if (GameManager.getAllianceColor() == Alliance.Blue){
            if (shooterPose.getX() > 5.0)
                return ShotType.Feed;
            return ShotType.Hub;
        } else if(shooterPose.getX() < 11.0)
                return ShotType.Feed;
            return ShotType.Hub;
    }

    public static Rotation2d getTargetRotation(Pose2d shooterPose, ShotType shotType){
        return getTargetPose(shooterPose, shotType).getTranslation().minus(shooterPose.getTranslation()).getAngle();
    }

    public static double getTargetDistance(Pose2d shooterPose, ShotType shotType){
        return getTargetPose(shooterPose, shotType).getTranslation().getDistance(shooterPose.getTranslation());
    }

    public static Pose2d getHubPose(){
        Pose2d hubPos;
        if (GameManager.getAllianceColor() == Alliance.Blue){
            hubPos = blueHubPos;
        } else{
            hubPos = redHubPos;
        }
        return hubPos;
    }

    public static Pose2d getFeedPose(Pose2d shooterPose){
        if (GameManager.getAllianceColor() == Alliance.Blue){
            if (shooterPose.getY() > yMiddleThreshold)
                return blueFeedTopPos;
            return blueFeedBottomPos;
        }
        if (shooterPose.getY() > yMiddleThreshold)
                return redFeedTopPos;
        return redFeedBottomPos;
    }
}

