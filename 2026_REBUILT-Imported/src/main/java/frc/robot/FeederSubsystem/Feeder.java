package frc.robot.FeederSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//includes conveyor system and serializer
public class Feeder extends SubsystemBase{
    TalonFX feeder = new TalonFX(FeederConstants.kCanID);
    

    public Feeder(){
        feeder.getConfigurator().apply(FeederConstants.getConfig());
    }

    public void setVoltage(double voltage){
        feeder.setVoltage(voltage);
    }

    public Command runFeeder(double voltage){
        return Commands.runOnce(()-> setVoltage(voltage));
    }


}
