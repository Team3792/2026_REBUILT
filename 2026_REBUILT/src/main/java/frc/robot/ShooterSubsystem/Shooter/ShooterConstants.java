// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ShooterSubsystem.Shooter;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

/** Add your docs here. */
public class ShooterConstants {
    //pid
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFrictionFF = 0;

    public static TalonFXConfiguration applyRightMotorConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
         config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;//TO-DO; make accurate
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        return config;
    }
    public static TalonFXConfiguration applyLeftMotorConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
         config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;//TO-DO; make accurate
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        return config;
    }

    private static final double kFlywheelDiameterInches = 4; //diameter in inches
    public static final double kFlywheelDiameter = kFlywheelDiameterInches * 0.0254; //convert inches to meters
    public static final double kFlywheelRadius = kFlywheelDiameter / 2;

    public static final double maxShootDistanceInch = 241.65;
    public static final double bumperThicknessInch = 5;
    public static final double shooterToFrontRobotInch = 20;
    public static final double maxRobotShootDistanceInch = maxShootDistanceInch - bumperThicknessInch - shooterToFrontRobotInch;


    //distance vs rpm 2D array
        //TODO set good rpm values for given range
    public static double[][] distanceVSRPM = {{24,48,72,96,120,144,168,192,216,240,264},{0,0,0,0,0,0,0,0,0,0,0}};
    //distance vs hood 2D array
        //TODO set good hood angle values for given range
    public static double[][] distanceVSHoodAngle = {{24,48,72,96,120,144,168,192,216,240,264},{0,0,0,0,0,0,0,0,0,0,0}};




}
