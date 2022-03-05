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
    //775 pro motor

    motor.configContinuousCurrentLimit(MotorConfigs.vexContinuousCurrentLimit, 0);
		motor.configPeakCurrentLimit(MotorConfigs.vexPeakCurrent, 0);
		motor.configPeakCurrentDuration(MotorConfigs.vexPeakDuration, 0);
		motor.enableCurrentLimit(true);	
    motor.configOpenloopRamp(0, 0);
    //psotive should be to shoot
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

    //|| motor.getSensorCollection().isRevLimitSwitchClosed()
    // SmartDashboard.putBoolean("isClosed", motor.getSensorCollection().isRevLimitSwitchClosed());

  }


  //angles of cllector - top - 27*, bottom - -4* , collecting - 30*

 // - level - 59
 // locked - 65 which is 5 over level - erorr rounding?

 // at bottom - -3


  //an srx controlled motor
  public WPI_TalonSRX motor = new WPI_TalonSRX(RobotMap.intakeOutake);  

  // DigitalInput toplimitSwitch = new DigitalInput(18);

  public void dump() {
    motor.set(0.6);
    // leftShooter.set(ControlMode.Velocity, rpmtoMeasuredUnits(MotorConfigs.shooterTargetVel));
  }
  public void collect() {
    // leftShooter.set(ControlMode.Velocity, rpmtoMeasuredUnits(MotorConfigs.shooterTargetVel));
    motor.set(-0.6);
  }

  public void stop() {
    motor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
