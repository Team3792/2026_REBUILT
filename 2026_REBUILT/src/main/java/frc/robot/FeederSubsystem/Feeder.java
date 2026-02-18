package frc.robot.FeederSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;

//includes conveyor system and serializer
public class Feeder extends SubsystemBase{
    TalonFX kicker = new TalonFX(HardwareMap.kickerID);
    TalonFX Conveyor = new TalonFX(HardwareMap.conveyorID);

    double a;
    

    public Feeder(){
        kicker.getConfigurator().apply(FeederConstants.getKickerConfig());
        Conveyor.getConfigurator().apply(FeederConstants.getConveyorConfig());
    }

    public void setKickerVoltage(double voltage){
        kicker.setVoltage(voltage);
    }

    public void setConveyorVoltage(double voltage){
        Conveyor.setVoltage(voltage);
    }

    public void setSubsystemVoltage(){
        setKickerVoltage(FeederConstants.kKickerVoltage);
        setConveyorVoltage(FeederConstants.kConveyorVoltage);
    }

    public Command runKicker(double voltage){
        return Commands.runOnce(()-> setKickerVoltage(voltage));
    }

    public Command runConveyor(double voltage){
        return Commands.runOnce(() -> setConveyorVoltage(voltage));
    }

    //run both conveyor, and kicker
    public Command runFeederSubsystem(){
        return Commands.parallel( runKicker(FeederConstants.kKickerVoltage),
                                  runConveyor(FeederConstants.kConveyorVoltage));
    }


}
