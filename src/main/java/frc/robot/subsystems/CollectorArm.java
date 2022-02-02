// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CollectorArm extends SubsystemBase {
  /** Creates a new CollectorArm. */
  public CollectorArm() {

    

  }

  //an srx controlled motor
  public WPI_TalonSRX motor = new WPI_TalonSRX(RobotMap.collectorArm);



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
