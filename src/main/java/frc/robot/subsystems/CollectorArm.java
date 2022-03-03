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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MotorConfigs;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class CollectorArm extends SubsystemBase {

  RobotContainer container;

  //if set point lower than current amp up kD?
  PIDController localPID = new PIDController(0.1, 0.01, 0.05);
  double previusPrediction = 0;

  double setPoint = 0;

  /** Creates a new CollectorArm. */
  public CollectorArm(RobotContainer container) {
    this.container = container;
    motor.setIdleMode(IdleMode.kBrake);

    motor.setSmartCurrentLimit(MotorConfigs.neoCurrentLimit);


    

    // PID coefficients
    //look into changing coefs based on ange - low angles have higher ones
    kP = 0.1;  //can i reduce this? 
    kI = 1e-2;
    kD = 0.05;//0.05 works ok 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0.8; 
    kMinOutput = -0.8;

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
     SmartDashboard.putNumber("Set Rotations", 15);
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
    
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    //if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { pid.setP(p); kP = p; }
    if((i != kI)) { pid.setI(i); kI = i; }
    if((d != kD)) { pid.setD(d); kD = d; }
    if((iz != kIz)) { pid.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { pid.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      pid.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 


    }

    // double rotations = 10;
    
      if (container.rightButton.get()) {
        pid.setIAccum(0);
        localPID.reset();
        // pid.set
      }

    SmartDashboard.putNumber("thruttle", container.xbox.getRawAxis(container.xboxRightTrigger));
    
    if (container.xbox.getRawAxis(container.xboxRightTrigger) > 0.8) {
      pid.setReference(rotations, CANSparkMax.ControlType.kPosition);
      
      double predict = localPID.calculate(motor.getEncoder().getPosition(), rotations);
      
      
     //clmap to [-0.6,0.6]
     predict = Math.min(predict,0.6);
     predict = Math.max(predict,-0.6);

      double actualPredict = lerp(predict, previusPrediction,0.5);
      previusPrediction = actualPredict;
      //motor.set(actualPredict);
    }




    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", motor.getEncoder().getVelocity());

    SmartDashboard.putNumber("current I windup", pid.getIAccum());
    // SmartDashboard.putNumber("current D windup", pid.setd());

      SmartDashboard.putNumber("Curent", motor.getOutputCurrent());

  }

  ///moves pid set point pid driving motor hapens in periodic fucntion
  public void pidMove(double val) {


  }


  public void manualMove(double val) {
    motor.set(-val);
  }

  public double lerp(double a, double b, double t) {
    return a + (b-a) * t;
  }
}


