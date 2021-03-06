// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class ManualDrive extends CommandBase {

  RobotContainer m_container;
  Drivetrain m_drivetrain;

  /** Creates a new ManualDrive. */
  public ManualDrive(RobotContainer container) {
    this.m_drivetrain = container.m_drivetrain;
    this.m_container = container;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(container.m_drivetrain);
    
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = m_container.joystick.getX();
    double y = m_container.joystick.getY();
    
    if (m_container.button3.get()) {
      x = 0;
    }
    
    double reduceMultiplier = 0.8;
    if (m_container.button6.get()) {
      x *= reduceMultiplier;
    }



    if (m_container.button5.get()) {
      y *= reduceMultiplier;
    }
    
    if (m_container.button4.get()) {
      y = 0;
    }
    
    double sensativity = -m_container.joystick.getThrottle();

    //remap from [-1,1] to [0,1]
    sensativity = (sensativity + 1.0) / 2.0;

    double minSens = 0.4;
    double maxSens = 1;

    //remap from [0,1] to [minSens,maxSens]
    sensativity = minSens + (sensativity - 0) * (maxSens - minSens) / (1 - 0);
    
    SmartDashboard.putNumber("sensativity", sensativity);


    m_drivetrain.drive(y * sensativity,x * sensativity * 0.9);

    if (m_container.aButton.get())
      m_drivetrain.resetEncoders();
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
