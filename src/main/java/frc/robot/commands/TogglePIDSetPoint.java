// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Collection;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CollectorArm;

public class TogglePIDSetPoint extends CommandBase {

  

  CollectorArm collector;

  /** Creates a new TogglePIDSetPoint. */
  public TogglePIDSetPoint(RobotContainer container) {
    collector = container.m_Collection;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // boolean ran = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (ran) {return;}
    collector.toggleSetPoint(); 
    // ran = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
