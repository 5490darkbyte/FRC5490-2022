// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorConfigs;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

/*
The collector amr uses two coordinate spaces and two units:
Units:
  Native Units: native rotations of neo motor - encoder postion is outputed in these units
  degrees - degres of arm
    use degreeToNative and nativeToDegree to convert
  Coordinate Systems:
  absolute - origin = arm lowered position
  floated - exact orign moves but zero in floated is zero from encoder.getPosition()
*/
public class CollectorArm extends SubsystemBase {
  

  // - level - 59
  // locked - 65 which is 5 over level - erorr rounding?
  
 // at bottom - -3

 //all units in absolute from bottom stop

//  public final double topStop = degreeToNative(65 + 3 + 4) - startRotOffset;
//  public final double shootingStop = degreeToNative(60 + 3 - 3) - startRotOffset;
//  public final double bottomStop = degreeToNative(0) - startRotOffset;
  

//For resetting floating origin - change constants to not reference floating origin

//MARK - v2

// public final double topStop = degreeToNative(59) - startRotOffset;
// //  public final double shootingStop = degreeToNative(60 + 3 - 3) - startRotOffset;
//  public final double bottomStop = degreeToNative(-5) - startRotOffset;

// public final double topDeadzoneStart = degreeToNative(59 + 2) - startRotOffset;
// public final double topHardStop = degreeToNative(59 + 4) - startRotOffset;
// public final double topHardStopValue = -0.042;

//MARK - v3

//offsetPositions - in absolute native coords

public final double topStop = degreeToNative(107);
public final double shootingStop = degreeToNative(98);
public final double bottomStop = degreeToNative(0);

public final double topDeadzoneStart = degreeToNative(107 + 2);
public final double topDeadzoneBuffer = degreeToNative(0.5);
public final double topHardStop = degreeToNative(107 + 4);
public final double topHardStopValue = -0.042 * 2;

public final double minFFAngle = 10;

  RobotContainer container;

  //if set point lower than current amp up kD?
  PIDController localPID = new PIDController(0.03, 0, 0.0);//0.01);
  double previusPrediction = 0;
  double previusManual = 0;

  //in absolute native units
  double setPoint = topStop;//topStop;//17;//15

  boolean setDown = false;



  //first number is relative angle from bottm reference point
  //second +3 is because - bottom is -3 degrees
  //+4 is unknown will probably be removed

  // a.k.a. floating origin position -
  //subtract absolute value by this to convert to floated
  //add floated value to this to get absolute
  double startRotOffset = topStop;// degreeToNative(65+3+4);
  
  public double safetyClampted(double speed,boolean manual) {
    double outSpeed = speed;
    SmartDashboard.putNumber("current native absolute",getNativeAbsolutePosition());
    if (getNativeAbsolutePosition() > topDeadzoneStart) {
      if (outSpeed > 0) {
        outSpeed = lerp(0, speed, 0.2);
      }
    }
    if (getNativeAbsolutePosition() > topHardStop) {
      outSpeed = Math.min(topHardStopValue,outSpeed);
    }

    //if lower limit tripped - reset floating origin and if going down or pid holding down stop motor
    if (!lowerLimit.get()) {
      //reset origin
    // TODO: rest pid to clear i term
      //startRotOffset is in absolute
      startRotOffset = 0;
      motor.getEncoder().setPosition(0);
      // startRotOffset = degreeToNative(nativePosition()) + topStop;

      if (outSpeed < 0) {
        outSpeed = 0;
      }
      //TODO: fix bottom stop because shouldnt it be absolute zero
      if (setPoint <= bottomStop && !manual) {
        outSpeed = 0;
      }
    }

    return outSpeed;
  }


  /** Creates a new CollectorArm. */
  public CollectorArm(RobotContainer container) {
    this.container = container;

    // motor.setIdleMode(IdleMode.kBrake);//remove when added to spark max
    //call stop here 
    //stop() - un comment wonce spark max changed

    motor.setSmartCurrentLimit(MotorConfigs.neoCurrentLimit);
    motor.setOpenLoopRampRate(0);
    motor.setClosedLoopRampRate(0);

    //asuming arm is at top
    // toggleSetPoint();
    // setPoint = topStop;
  }

  //a spark max controlled motor
  // public WPI_TalonSRX motor = new WPI_TalonSRX(RobotMap.collectorArm);
  
  //WARNING: if motor chanes to brushed this line must be updatee
  CANSparkMax motor = new CANSparkMax(RobotMap.collectorArm,MotorType.kBrushless);
  // SparkMaxPIDController pid = motor.getPIDController();
  DigitalInput lowerLimit = new DigitalInput(RobotMap.lowerLimitSWH);

  
  //in native units and floated coords
  public double nativePosition() {
    return motor.getEncoder().getPosition();
  }

   //in native units and floated coords
   public double getNativeAbsolutePosition() {
    return nativePosition() + startRotOffset;
  }

  //in degres units and absolute coords
  public double getCurrentAbsoluteAngle() {
    return nativeToDegree(getNativeAbsolutePosition());
  }

  @Override
  public void periodic() {
    //in floated native coords
    double rotations = setPoint - startRotOffset;
    // double rotations = 10;
    
      if (container.rightButton.get()) {
        // pid.setIAccum(0);
        localPID.reset();
        // pid.set
      }

    SmartDashboard.putNumber("thruttle", container.xbox.getRawAxis(container.xboxRightTrigger));
    
    if (container.xbox.getRawAxis(container.xboxRightTrigger) < 0.8) {

      double predict = localPID.calculate(nativePosition(), rotations);
      double feedBias = 0;
      if (setPoint == shootingStop) {
        feedBias *= 1.2;
      }else {
        feedBias *= 0.9;
      }
      
        predict += feedForwardForAngle(getCurrentAbsoluteAngle(),feedBias);
      // }

      //clmap to [-0.6,0.6]
      predict = Math.min(predict,0.37);
      predict = Math.max(predict,-0.32);

      predict = safetyClampted(predict,false);

      double actualPredict = lerp(previusPrediction,predict,0.5);//0.5);
      previusPrediction = actualPredict;
      SmartDashboard.putNumber("PIDsendingValue", actualPredict);

      //DO NOT UNCOMENT UNTIL CHANGING ALL UNITS
      motor.set(actualPredict);
    }





    SmartDashboard.putNumber("FloatedSetPoint", rotations);

    // SmartDashboard.putNumber("current I windup", pid.getIAccum());
    // SmartDashboard.putNumber("current D windup", pid.setd());

    SmartDashboard.putNumber("CurentCurrent", motor.getOutputCurrent());

    SmartDashboard.putNumber("currentArmRot", nativeToDegree(motor.getEncoder().getPosition()));
    SmartDashboard.putNumber("currentArmRotFromAbsolute", nativeToDegree(motor.getEncoder().getPosition() + startRotOffset));



    //limit
    SmartDashboard.putBoolean("newLowerLimit2", lowerLimit.get());

  }


  public void manualMove(double val) {
    if (container.xbox.getRawAxis(container.xboxRightTrigger) > 0.8) {

      double reduction = 0.4;//0.9;

      // if (container.rightButton.get()) {
      //   reduction = 1;
      // }

        double power = -val * reduction;
        
        // if (getCurrentAbsoluteAngle() > minFFAngle) {
          if (!container.leftButton.get()) {
            power += feedForwardForAngle(getCurrentAbsoluteAngle(),0);
          }
        // }

        power = lerp(previusManual, power, 0.05);
        previusManual = power;
        
        power = safetyClampted(power,true);
        SmartDashboard.putNumber("outSent",power);
        motor.set(power);
    }
  }

  //takes in angle in absolute degrees
  public double feedForwardForAngle(double angle,double powerThrough) {
    double maxPower = 0.104688 * 0.9; //power at zero degrees
    double minPower = 0.01;//0.025; //power at degree max
    double maxAngle = 80;
    double diff = maxPower - minPower;

    double fadoutSize = 5;
    double minFadout = 4;
    double upperFadout = Math.max(Math.min(-angle + maxAngle + fadoutSize,1),0);
    double lowerFadout = Math.max(Math.min(angle - minFadout - fadoutSize,1),0);

    // if (angle > maxAngle) {
    //   return 0;
    // }

    double ff = Math.cos(angle * Math.PI / 2 / maxAngle) / 2 * diff + diff / 2 + minPower;

    return ff * lowerFadout * upperFadout * powerThrough;
  }

  public double lerp(double a, double b, double t) {
    return a + (b-a) * t;
  }
  
  public double nativeToDegree(double nat) {
    return (nat / (20 * 5)) * 360;
  }
  public double degreeToNative(double deg) {
    return deg / 360 * (20 * 5);
  }


  public void toggleSetPoint() {
    if (setPoint == topStop) {
      setPoint = bottomStop;
      // localPID.setD(0.005);
      // localPID.setI(0.001);
      // localPID.setP(0.02);
      setDown = true;
    } else {
      setPoint = topStop;
      // localPID.setD(0.01);  
      // localPID.setI(0.005);
      // localPID.setP(0.08);
      setDown = false;
    }
  }
}