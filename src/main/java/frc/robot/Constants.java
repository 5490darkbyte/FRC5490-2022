// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //MARK: Drivetrain

    //TODO: fill out correct dimension
    public static double robotDriveWidthInches = 26.564;

    /// multiply motor speed by this to get wheel speed
    //whels are spining 9 times slower than the motors
    public static double gearRatio = 1.0/9.0;
    /// in meters
    public static double wheelRadius = Units.inchesToMeters(2);


    // characterization constants
    public static double ks = 0;
    public static double kv = 0;
    public static double ka = 0;

    public static double drivetrainPIDKp = 0.1;
    

    //MARK: Cargo Intake/Outake
    public static double collectorArmGearRatio = 1.0 / 80.0;

    public static double intakeOutakeGearRatio = 1.0 / 7.0;
}
    