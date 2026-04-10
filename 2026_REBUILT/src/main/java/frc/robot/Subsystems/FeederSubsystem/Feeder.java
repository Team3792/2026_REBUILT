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
    TalonFX conveyor = new TalonFX(HardwareMap.conveyorID.id());

    double a;
    

    public Feeder(){
        kicker.getConfigurator().apply(FeederConstants.getKickerConfig());
        conveyor.getConfigurator().apply(FeederConstants.getConveyorConfig());
    }

    public void setKickerVoltage(double voltage){
        kicker.setVoltage(voltage);
    }

    public void setConveyorVoltage(double voltage){
        conveyor.setVoltage(voltage);
    }

    public void setVoltage(double conveyor, double kicker){
        setConveyorVoltage(conveyor);
        setKickerVoltage(kicker);
    }

    public Command feed(double conveyorVoltage, double kickerVoltage){
        return this.startEnd(
            () -> setVoltage(conveyorVoltage, kickerVoltage), 
            () -> setVoltage(0, 0)
        );
    }

    public Command feed(){
        return feed(FeederConstants.kConveyorVoltage, FeederConstants.kKickerVoltage);
    }
}
