// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MotorConfigs;
import frc.robot.RobotMap;
import pabeles.concurrency.IntOperatorTask.Min;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

public class Drivetrain extends SubsystemBase {

  //#region Members  

  //controller docs at: https://store.ctr-electronics.com/content/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_talon_f_x.html 
  // or https://store.ctr-electronics.com/falcon-500-powered-by-talon-fx/
  //TODO: fix imidiatly lefts are actually right
  WPI_TalonFX leftBack = new WPI_TalonFX(RobotMap.backLeftDrive);
  WPI_TalonFX leftFront = new WPI_TalonFX(RobotMap.frontLeftDrive);

  MotorControllerGroup lefts = new MotorControllerGroup(leftBack,leftFront);
  
  WPI_TalonFX rightBack = new WPI_TalonFX(RobotMap.backRightDrive);
  WPI_TalonFX rightFront = new WPI_TalonFX(RobotMap.frontRightDrive);
  
  MotorControllerGroup rights = new MotorControllerGroup(rightBack,rightFront);

  DifferentialDrive differentialDrive = new DifferentialDrive(lefts,rights);

  WPI_Pigeon2 gyro = new WPI_Pigeon2(RobotMap.gyro);

  //#endregion
  
  DifferentialDriveKinematics kinimatics = new DifferentialDriveKinematics(Units.inchesToMeters(Constants.robotDriveWidthInches));
  DifferentialDriveOdometry odometry;
  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.ks, Constants.kv, Constants.ka);

  PIDController leftPidController = new PIDController(Constants.drivetrainPIDKp, Constants.drivetrainPIDKi, Constants.drivetrainPIDKd);
  PIDController rightPidController = new PIDController(Constants.drivetrainPIDKp, Constants.drivetrainPIDKi, Constants.drivetrainPIDKd);


  
  
  /** Creates a new Drivetrain. */
  public Drivetrain() {

    //configure motor limits

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = MotorConfigs.universalCurrentLimit; // the peak supply current, in amps
    config.supplyCurrLimit.triggerThresholdTime = MotorConfigs.universalPeakDuration; // the time at the peak supply current before the limit triggers, in sec
    config.supplyCurrLimit.currentLimit = 30; // the current to maintain if the peak supply limit is triggered
    
    
    leftBack.configAllSettings(config);
    leftFront.configAllSettings(config);  

    rightBack.configAllSettings(config);
    rightFront.configAllSettings(config);

    //left1.sensor

    rights.setInverted(true);

    //trajectory tracing
    resetEncoders();
    odometry = new DifferentialDriveOdometry(getHeading(),initialPosiiton());

  }


  public void resetOdometry(Pose2d pos) {
    resetEncoders();
    odometry.resetPosition(pos, getHeading());
  }

  public void resetEncoders() {
    leftBack.setSelectedSensorPosition(0);
    leftFront.setSelectedSensorPosition(0);

    rightBack.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
  }

  public void resetPID() {
    leftPidController.reset();
    rightPidController.reset();
  }


  //in motor rpm
  public double averagedLeftEncoderVelocity() {
    double average = 
    (leftBack.getSelectedSensorVelocity() +
    leftFront.getSelectedSensorVelocity()) / 2;

    return measuredUnitsTorpm(average);
  }

  //in motor rpm
  public double averagedRightEncoderVelocity() {
    double average = 
    (rightBack.getSelectedSensorVelocity() +
    rightFront.getSelectedSensorVelocity()) / 2;

    return measuredUnitsTorpm(average);
  }

  //in meters
  public double averagedLeftEncoderPos() {
    double average = 
    (leftBack.getSelectedSensorPosition()  +
    leftFront.getSelectedSensorPosition()) / 2;

    return measuredPosToMeters(average);//measuredUnitsTorpm(average);
  }

  //in meters
  public double averagedRightEncoderPos() {
    double average = 
    (rightBack.getSelectedSensorPosition() +
    rightFront.getSelectedSensorPosition()) / 2;

    return measuredPosToMeters(average);//measuredUnitsTorpm(average);
  }




  public Pose2d initialPosiiton() {
    return new Pose2d(0,0,initialRotation());
  }

  public Rotation2d initialRotation() {
    return new Rotation2d(0);
  }

  ///ccw = positive
  public Rotation2d getHeading() {
    //TODO: fix
    return Rotation2d.fromDegrees(0);
  }

  /// in m/s
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      averagedLeftEncoderVelocity() * Constants.gearRatio * 2 * Math.PI * Constants.wheelRadius / 60,
      averagedRightEncoderVelocity() * Constants.gearRatio * 2 * Math.PI * Constants.wheelRadius / 60
      );
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Pose2d currentPos = odometry.update(getHeading(), averagedLeftEncoderPos(), averagedRightEncoderPos());

    SmartDashboard.putNumber("pos_x", getWheelSpeeds().leftMetersPerSecond);//currentPos.getX());
    SmartDashboard.putNumber("pos_y", currentPos.getY());
    SmartDashboard.putNumber("pos_zRot", currentPos.getRotation().getDegrees());
  }



  //MARK: Manual Driving

  //negative zRot id counter clockwise rotation
  public void drive(double xSpeed, double zRot) {
    differentialDrive.arcadeDrive(xSpeed, -zRot);
    //lefts.set(xSpeed);
    //lefts.set(0.3);
  }

  public void pidDrive(double setPoint) {
    //just use left encoders to fake it

    double prediciton = leftPidController.calculate(averagedLeftEncoderPos(), setPoint);

    SmartDashboard.putNumber("drivetrainEncoderVal", prediciton);

    //clmap to [-0.5,0.5]
    prediciton = Math.min(prediciton,0.4);
    prediciton = Math.max(prediciton,-0.4);

    differentialDrive.arcadeDrive(prediciton, 0);
  }

  public void stop() {
    differentialDrive.arcadeDrive(0,0);
  }

  //Conversion Functions

  double measuredPosToMeters(double measured) {
    return measured / 2048.0 * Constants.gearRatio * Constants.wheelRadius * Math.PI * 2;
  }

  double metersToMeasuredPos(double meters) {
    return meters * 2048.0 / Constants.gearRatio / Constants.wheelRadius / Math.PI / 2;
  }

  //TODO: check both of these conversion functions might only be for last years shooter
  double measuredUnitsTorpm(double measured) {
    return (measured / 4096) * 600;
  }
  
  //TODO: check both of these conversion functions
  double rpmtoMeasuredUnits(double rpm) {
    return (rpm / 600) * 4096;
  } 

  // double RotationsPosTo

}
