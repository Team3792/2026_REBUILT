// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class JoystickInput {
    double max, deadband;

    public JoystickInput(double max, double deadband){
        this.max = max;
        this.deadband = deadband;
    }

    public boolean inDeadband(double value){
        return Math.abs(value) <= deadband;
    }

    public double get(double value){
        //Apply deadband first, then scale
        if(inDeadband(value))
            return 0;
        return value * max;
    }
}
