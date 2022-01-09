// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Drivetrain extends SubsystemBase {

  //https://www.vexrobotics.com/pro/falcon-500

  TalonFX left1 = new TalonFX(RobotMap.backLeftDrive);
  TalonFX left2 = new TalonFX(RobotMap.frontLeftDrive);

  TalonFX right1 = new TalonFX(RobotMap.backRightDrive);
  TalonFX right2 = new TalonFX(RobotMap.frontRightDrive);

  /** Creates a new Drivetrain. */
  public Drivetrain() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
