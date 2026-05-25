package frc.robot.util;

public class HardwareMap {
    public static final int kDrivePort = 0;
    public static final int kOperatorPort = 1;

    public static final int kPDH = 1;

    //drive
    public static final CANAddress kPigeon = new CANAddress(12, "pigeon", 0);

    //intake
    public static final CANAddress intakePivotID = new CANAddress(20, "intakePivot", 0);
    public static final CANAddress intakeDriveLeftID = new CANAddress(21,"intakeDrive", 0);
    public static final CANAddress intakeDriveRightID = new CANAddress(22, "intakeDrive",0);

    //feeder
    public static final CANAddress conveyorID = new CANAddress(23, "conveyor", 0);
    public static final CANAddress kickerID = new CANAddress(24, "kicker", 0);

    //shooter & hood
    public static final CANAddress shootLeaderID = new CANAddress(30, "shootLeader", 0);
    public static final CANAddress shootFollowerID = new CANAddress(31, "shootFollower", 0);                                                                                                           
    public static final int hoodID = 32;
    public static final int hoodEncoderPort = 1;

    public record CANAddress(int id, String name, int index){}
}
