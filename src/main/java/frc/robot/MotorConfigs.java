package frc.robot;

public class MotorConfigs
{
    public static final int universalCurrentLimit = 39;
    public static final int universalPeakDuration = 100;
    // VEX Robotics CIM Motors
    public static final int vexContinuousCurrentLimit = universalCurrentLimit;
    public static final int vexPeakCurrent = 60;
    public static final int vexPeakDuration = universalPeakDuration;

    // VEX Robotics 775pro motor (smaller than VEX CIM motor)
    public static final int vexSmallContinuousCurrentLimit = 20;
    public static final int vexSmallPeakCurrent = 25;
    public static final int vexSmallPeakDuration = universalPeakDuration;

    // Andymark Redline Motors
    public static final int redlineContinuousCurrentLimit = universalCurrentLimit;
    public static final int redlinePeakCurrent = 55;
    public static final int redlinePeakDuration = universalPeakDuration;

    public static final double redlineLeftCruiseVelocity = 58617.36;
    public static final double redlineRightCruiseVelocity = 66768.0;

    public static final double redlineLeftAccel = 58617.36;
    public static final double redlineRightAccel = 66768.0;

    // 

    //NEO Motor
    public static final int neoCurrentLimit = 80;//universalCurrentLimit;
    
}