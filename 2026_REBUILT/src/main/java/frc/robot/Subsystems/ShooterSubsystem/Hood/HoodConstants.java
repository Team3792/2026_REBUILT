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

    public static final double kP = 0.2;
    public static final double kI = 0;//0.0001;
    public static final double kD = 0.005;
    public static final double kFrictionFF = 0;
    public static final double kMotorRatio = 25;
    public static final double kHoodRatio = 300.0/26;

    // All in degrees
    public static final double kMinAngle = 12;
    public static final double kMaxAngle = 50;
    public static final double kZeroAngle = 12;

    public static final double kAngleTolerance = 0.25;

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

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.maxOutput(0.8);
        config.closedLoop.minOutput(-0.8);
   
        //config.closedLoop.iMaxAccum(0.2);
        return config;
    }

    

    public static final double kHubHoodAngle = 20;
}
