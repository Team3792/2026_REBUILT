package frc.robot.Subsystems.FeederSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap;

//includes conveyor system and serializer
public class Feeder extends SubsystemBase{
    TalonFX kicker = new TalonFX(HardwareMap.kickerID.id());
    TalonFX Conveyor = new TalonFX(HardwareMap.conveyorID.id());

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

    public void setSubsystemVoltage(double voltage1, double voltage2){
        setKickerVoltage(voltage1);
        setConveyorVoltage(voltage2);
    }

    public Command runKicker(double voltage){
        return Commands.runOnce(()-> setKickerVoltage(voltage));
    }

    public Command runConveyor(double voltage){
        return Commands.startEnd(() -> setConveyorVoltage(voltage), () -> setConveyorVoltage(0));
    }

    public Command runFeeder(double kicker, double belts){
        return Commands.startEnd(() -> setSubsystemVoltage(kicker, belts), () -> setSubsystemVoltage(0, 0));
    }

    //run both conveyor, and kicker
    public Command runFeederSubsystem(){
        return Commands.parallel( runKicker(FeederConstants.kKickerVoltage),
                                  runConveyor(FeederConstants.kConveyorVoltage));
    }


}
