// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorConfigs;
import frc.robot.RobotMap;

public class IntakeOuttake extends SubsystemBase {
  /** Creates a new IntakeOuttake. */
  public IntakeOuttake() {

    // motor.configContinuousCurrentLimit(MotorConfigs.redlineContinuousCurrentLimit, 0);
		// motor.configPeakCurrentLimit(MotorConfigs.redlinePeakCurrent, 0);
		// motor.configPeakCurrentDuration(MotorConfigs.redlinePeakDuration, 0);
		// motor.enableCurrentLimit(true);	
    // motor.configOpenloopRamp(0, 0);
    // motor.setInverted(true); // invert left shooter motor

    // //Motionmagic extra configurations
    // motor.configMotionCruiseVelocity(MotorConfigs.redlineLeftCruiseVelocity);
    // motor.configMotionAcceleration(MotorConfigs.redlineLeftAccel);

    // //pid values
    // motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 150);
    // motor.selectProfileSlot(0, 0);

    // //TODO: extract coeficients to constants
    // motor.config_kP(0, 0.015);
    // motor.config_kI(0, 0.00004);
    // motor.config_IntegralZone(0, 5000);
    // motor.config_kD(0, 0.3);
    // motor.config_kF(0,0);


  }

  //an srx controlled motor
  //public WPI_TalonSRX motor = new WPI_TalonSRX(RobotMap.intakeOutake);

  public void dump() {
    // leftShooter.set(ControlMode.Velocity, rpmtoMeasuredUnits(MotorConfigs.shooterTargetVel));
  }
  public void collect() {
    // leftShooter.set(ControlMode.Velocity, rpmtoMeasuredUnits(MotorConfigs.shooterTargetVel));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
