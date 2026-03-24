// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.ShooterSubsystem.Shooter;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

/** Add your docs here. */
public class ShooterConstants {

    //TODO make accurate
    public static final double kLightSensorThreshold = 0.5;
    public static final double lightSensorDetectionRange = 0.01;//meters
    public static final double kballSeperationTime = 0.1;
    public static final double kFlywheelBallLoadCurrent = 0;//amps

    //pid & FF
    public static final double kV = 0.1163;
    public static final double kS = 0.23;
    public static final double kMMAccel = 200;
    public static final double kMMJerk = 2100;
    public static final double kP = 0.1; 
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFrictionFF = 0;

    public static TalonFXConfiguration applyRightMotorConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 70;

        Slot0Configs slot0 = config.Slot0;
        slot0.kS = kS;
        slot0.kV = kV;
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;

        MotionMagicConfigs mmconfig = config.MotionMagic;
        mmconfig.MotionMagicAcceleration = kMMAccel;
        mmconfig.MotionMagicJerk = kMMJerk;
        return config;
    }
    public static TalonFXConfiguration applyLeftMotorConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 70;

        Slot0Configs slot0 = config.Slot0;
        slot0.kS = kS;
        slot0.kV = kV;
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;

        MotionMagicConfigs mmconfig = config.MotionMagic;
        mmconfig.MotionMagicAcceleration = kMMAccel;
        mmconfig.MotionMagicJerk = kMMJerk;
        return config;
    }

    private static final double kFlywheelDiameterInches = 4; //diameter in inches
    public static final double kFlywheelDiameter =  Units.inchesToMeters(kFlywheelDiameterInches); //kFlywheelDiameterInches * 0.0254; //convert inches to meters
    public static final double kFlywheelRadius = kFlywheelDiameter / 2;

    public static final double kFallBackHubVoltage = 6.0;

    public static final double maxShooterRPS = 6000;
    //rpm value for flywheel while not shooting; less time to spin up to target rpm
    public static final double coastRPM = 0;

    public static final double kHubRPS = 45;

    public record CalibratedShot(double distanceMeters, double omegaRPS, double hoodAngleDegrees){};


    //TODO: figure these out, must be sorted by distance (need not be sorted by other values)
    public static final CalibratedShot[] kCalibratedShots = {
        new CalibratedShot(1.726, 52, 24),
        new CalibratedShot(2.03, 53.5, 25.5),
        new CalibratedShot(2.47, 55, 27),
        new CalibratedShot(3.03, 56, 33),
        new CalibratedShot(3.43, 57, 34),
        new CalibratedShot(4.01, 59.5, 37),
        new CalibratedShot(4.5, 60, 40),
        new CalibratedShot(5.02, 64, 43),
        new CalibratedShot(6.1, 70, 43),
        new CalibratedShot(7.2, 75, 43)
    };

    // //distance vs rps 2D array
    //     //TODO: set good rps values for given distance range
    // public static final double[][] distanceVSRPS = {{24,48,72,96,120,144,168,192,216,240,264},
    //                                                 {0,0,0,0,0,0,0,0,0,0,0}};
    //     //TODO: set good hood angle values for given distance range
    // public static final double[][] distanceVsHoodAngle = {{24,48,72,96,120,144,168,192,216,240,264},
    //                                                       {0,0,0,0,0,0,0,0,0,0,0}};
    // //                                               r:1 2 3 4 5 6 7 8 9 10 11
    // public static final double[][] hoodAngleRanges = {{0,0,0,0,0,0,0,0,0,0,0}, 
    //                                                   {0,0,0,0,0,0,0,0,0,0,0}};

    //TODO: instead of 2, use best fit curve to return correct amount of iterations needed
    public static double targetHubPoseIterations(double robotVelocity){
        return robotVelocity * 2;
    }

    //TODO: make accurate
    public static final double ktimeVsDistance = 0; //how long it takes fuel to travel one meter
                                                               //meters
    public static  double fuelFlightTimeVsDistance(double robotToHubDistance){
        return robotToHubDistance * ktimeVsDistance;
    }
}
