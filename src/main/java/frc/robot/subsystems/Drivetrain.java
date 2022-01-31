// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorConfigs;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Drivetrain extends SubsystemBase {

  //https://www.vexrobotics.com/pro/falcon-500

  WPI_TalonFX left1 = new WPI_TalonFX(RobotMap.backLeftDrive);
  WPI_TalonFX left2 = new WPI_TalonFX(RobotMap.frontLeftDrive);

  MotorControllerGroup lefts = new MotorControllerGroup(left1,left2);
  
  // WPI_TalonFX right1 = new WPI_TalonFX(RobotMap.backRightDrive);
  // WPI_TalonFX right2 = new WPI_TalonFX(RobotMap.frontRightDrive);
  
  // MotorControllerGroup rights = new MotorControllerGroup(right1,right2);

  // DifferentialDrive differentialDrive = new DifferentialDrive(lefts);

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    //configure motor limits

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = MotorConfigs.universalCurrentLimit; // the peak supply current, in amps
    config.supplyCurrLimit.triggerThresholdTime = MotorConfigs.universalPeakDuration; // the time at the peak supply current before the limit triggers, in sec
    config.supplyCurrLimit.currentLimit = 30; // the current to maintain if the peak supply limit is triggered
    left1.configAllSettings(config);
    left2.configAllSettings(config);
    //TODO: uncomment these
    //right1.configAllSettings(config);
    //right2.configAllSettings(config);

    left1.setInverted(true);
  }

  public void drive(double xSpeed, double zRot) {
    // differentialDrive.arcadeDrive(xSpeed, zRot);
    lefts.set(xSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
