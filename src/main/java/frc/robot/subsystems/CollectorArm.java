// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MotorConfigs;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class CollectorArm extends SubsystemBase {

  RobotContainer container;

  /** Creates a new CollectorArm. */
  public CollectorArm(RobotContainer container) {
    this.container = container;
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(MotorConfigs.neoCurrentLimit);


    

    // PID coefficients
    kP = 0.1; 
    kI = 1e-2;
    kD = 0.05*0;//0.05 works ok 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0.6; 
    kMinOutput = -0.6;

    // set PID coefficients
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
    pid.setIZone(kIz);
    pid.setFF(kFF);
    pid.setOutputRange(kMinOutput, kMaxOutput);
    
    // pid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal);
    pid.setSmartMotionMaxAccel(1, 0);

    motor.setOpenLoopRampRate(0.1);
    motor.setClosedLoopRampRate(0);

     // display PID coefficients on SmartDashboard
     SmartDashboard.putNumber("P Gain", kP);
     SmartDashboard.putNumber("I Gain", kI);
     SmartDashboard.putNumber("D Gain", kD);
     SmartDashboard.putNumber("I Zone", kIz);
     SmartDashboard.putNumber("Feed Forward", kFF);
     SmartDashboard.putNumber("Max Output", kMaxOutput);
     SmartDashboard.putNumber("Min Output", kMinOutput);
     SmartDashboard.putNumber("Set Rotations", 0);
  }

  //a spark max controlled motor
  // public WPI_TalonSRX motor = new WPI_TalonSRX(RobotMap.collectorArm);
  
  //WARNING: if motor chanes to brushed this line must be updatee
  CANSparkMax motor = new CANSparkMax(RobotMap.collectorArm,MotorType.kBrushless);
  SparkMaxPIDController pid = motor.getPIDController();
  double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // // read PID coefficients from SmartDashboard
    // double p = SmartDashboard.getNumber("P Gain", 0);
    // double i = SmartDashboard.getNumber("I Gain", 0);
    // double d = SmartDashboard.getNumber("D Gain", 0);
    // double iz = SmartDashboard.getNumber("I Zone", 0);
    // double ff = SmartDashboard.getNumber("Feed Forward", 0);
    // double max = SmartDashboard.getNumber("Max Output", 0);
    // double min = SmartDashboard.getNumber("Min Output", 0);

    
    // double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    // if((p != kP)) { pid.setP(p); kP = p; }
    // if((i != kI)) { pid.setI(i); kI = i; }
    // if((d != kD)) { pid.setD(d); kD = d; }
    // if((iz != kIz)) { pid.setIZone(iz); kIz = iz; }
    // if((ff != kFF)) { pid.setFF(ff); kFF = ff; }
    // if((max != kMaxOutput) || (min != kMinOutput)) { 
    //   pid.setOutputRange(min, max); 
    //   kMinOutput = min; kMaxOutput = max; 


    // }

    double rotations = 10;
    
      if (container.xButton.get()) {
        pid.setIAccum(0);
      }

    if (container.yButton.get()) {
      pid.setReference(rotations, CANSparkMax.ControlType.kPosition);
    }

    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", motor.getEncoder().getVelocity());


      SmartDashboard.putNumber("Curent", motor.getOutputCurrent());

  }

  

  public void manualMove(double val) {
    motor.set(-val);
  }
}
