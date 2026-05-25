package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

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
