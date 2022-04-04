// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorConfigs;
import frc.robot.RobotMap;

public class IntakeOuttake extends SubsystemBase {
  /** Creates a new IntakeOuttake. */
  public IntakeOuttake() {
    motor.configContinuousCurrentLimit(MotorConfigs.vexContinuousCurrentLimit, 0);
		motor.configPeakCurrentLimit(MotorConfigs.vexPeakCurrent, 0);
		motor.configPeakCurrentDuration(MotorConfigs.vexPeakDuration, 0);
		motor.enableCurrentLimit(true);	
    motor.configOpenloopRamp(0.3, 0);
  }

  //an srx controlled 775 pro motor  
  public WPI_TalonSRX motor = new WPI_TalonSRX(RobotMap.intakeOutake);  

  public void dump() {
    motor.set(1);
  }
  public void collect() {
    motor.set(-0.75);
  }

  public void stop() {
    motor.set(0);
  }
}
