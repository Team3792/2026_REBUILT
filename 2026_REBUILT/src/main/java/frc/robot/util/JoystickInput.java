package frc.robot.util;

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
