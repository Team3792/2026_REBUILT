package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Managers.GameManager;
import frc.robot.util.HardwareMap.CANAddress;

import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.wpilibj.Alert;

import java.util.ArrayList;

public class CANManager {
    private static ArrayList<CANConnection> canConnections = new ArrayList<>();
    public static Alert diconnectionAlert = new Alert("CAN disconnect, see DEBUG", AlertType.kError);

    public static void addConnection(CANAddress address, BooleanSupplier connectedFunction){
        //Sort array for index
        for(int i = 0; i < canConnections.size(); i++){
            if(address.index() <= canConnections.get(i).address.index()){
                canConnections.add(i, new CANConnection(address, connectedFunction));
                return;
           }
        }
            
        canConnections.add(new CANConnection(address, connectedFunction));
    }

    public static void addConnection(CANAddress address, ParentDevice ctreDevice) {
        addConnection(address, ctreDevice::isConnected);
    }

    public static void updateConnections() {
        boolean allConnected = true;
        for (CANConnection c : canConnections) {
            boolean connected = c.connectedFunction().getAsBoolean();

            SmartDashboard.putBoolean("CANConnections new/" + "[" + c.address.index() +  "] " + c.address.name() + " id:" + c.address.id(), connected);
            if(!connected){
                allConnected = false;
            }
        }
        diconnectionAlert.set(!allConnected);
        GameManager.error = !allConnected;
    }

    public record CANConnection(CANAddress address, BooleanSupplier connectedFunction) {}
}
