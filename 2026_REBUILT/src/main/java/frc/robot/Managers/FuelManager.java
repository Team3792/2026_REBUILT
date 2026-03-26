// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Managers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.ShooterSubsystem.Shooter.ShooterConstants;
import frc.robot.Subsystems.SwerveSubsystem.Swerve;
import frc.robot.Subsystems.ShooterSubsystem.Shooter.Shooter;

/** Add your docs here. */
public class FuelManager extends SubsystemBase {

    Shooter shooter = new Shooter();

    Timer timer = new Timer();
    Timer time = new Timer();
    boolean ballShot;
    public static double ballsShot = 0;

    public double RPM(){
        return shooter.getRPS();
    }

    public boolean ballInShooter(){
        return shooter.lightSensorReading() > ShooterConstants.kLightSensorThreshold;
    }

    public double flywheelStatorCurrent(){
        return 0;//shooter.flywheelStatorCurrent();
    }

    public double getTangentialVelocity(){
        return shooter.getTangentialVelocity();
    }

    // public double targetFlywheelVelocity(Swerve swerve, Pose2d hubPose){
    //     return shooter.targetFlywheelVelocity(swerve, hubPose);
    // }
    //time it take for ball to pass through the light sensor detection range
    public double ballShootTime(){
        return ShooterConstants.lightSensorDetectionRange / getTangentialVelocity();
    }

    
    //if ball detected is in shooter, and ball is gone after the time it takes to shoot 1 ball; returns true
    public boolean ballShot(){
        if(ballInShooter()){
            timer.start();
            if(timer.hasElapsed(ballShootTime())){
            ballShot = !ballInShooter();
            }
        }
        timer.reset();
        return ballShot;
    }

    //return true if flywheel current < current of flywheel under load of a ball && if there has been a decrease if flywheel RPMs
    // public boolean ballShotCurrentDetection(Swerve swerve, Pose2d hubPose){
    // return (flywheelStatorCurrent() < ShooterConstants.kFlywheelBallLoadCurrent) 
    //         && (RPM() < targetFlywheelVelocity(swerve, hubPose) );
    // }

    public double ballsShot(){
    time.reset();
    time.start();
    if(ballShot()){
        time.hasElapsed(ShooterConstants.kballSeperationTime);
        if(!ballShot){
        ballsShot += 1;
        }
    }
    return ballsShot;
    }
}
