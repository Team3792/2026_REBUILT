// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ShooterSubsystem.Hood;

import com.revrobotics.spark.FeedbackSensor;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public class HoodConstants {

    public static final double kP = 0.075;
    public static final double kI = 0.0001;
    public static final double kD = 0.005;
    public static final double kFrictionFF = 0;
    public static final double kMotorRatio = 5;
    public static final double kHoodRatio = 300./26;
    public static final double kZeroPosition = 12;// degrees

    // All in degrees
    public static final double kMinAngle = 15;
    public static final double kMaxAngle = 50;
    public static final double kZeroAngle = 12;

    //Hood Angle Constants for Zones
    public static final double kZone1Angle = 0;

    public static SparkMaxConfig configHood() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(40);
        config.idleMode(IdleMode.kBrake);
        config.inverted(true);
        //config.softLimit.forwardSoftLimitEnabled(true);
        //config.softLimit.reverseSoftLimitEnabled(true);
        //config.softLimit.forwardSoftLimit(50);
        //config.softLimit.reverseSoftLimit(0);
        config.closedLoop.pid(kP, kI, kD);

        config.closedLoop.feedbackSensor(FeedbackSensor.kNoSensor);
        config.closedLoop.maxOutput(0.5);
        config.closedLoop.minOutput(-0.5);
   
        //config.closedLoop.iMaxAccum(0.2);
        return config;
    }

    

    public static final double kEncoderAngleOffset = 0; //need to make accurate
    public static final double hoodAngleBestFit = 0;//need to set to best fit curve of hood angle vs distance
    private static final double timePerAngle = 0.1;

        //time to stow hood in seconds
        public static double timeToStow(double robotVelocity){
            return robotVelocity * timePerAngle; //need to make accurate
        }

}
