// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/** Add your docs here. */
public class ControlConfig {
    public record PIDConfig(double kP, double kI, double kD){
        public PIDController getController(){
            return new PIDController(kP, kI, kD);
        }
    };

    public record ProfiledPIDConfig(double kP, double kI, double kD, double maxV, double maxA){
        public ProfiledPIDController getController(){
            return new ProfiledPIDController(kP, kI, kD, new Constraints(maxV, maxA));
        }
    };

}
