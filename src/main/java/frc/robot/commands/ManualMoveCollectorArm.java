// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ManualMoveCollectorArm extends CommandBase {

  RobotContainer m_container;

  /** Creates a new ManualMoveCollectorArm. */
  public ManualMoveCollectorArm(RobotContainer container) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_container = container;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed = m_container.xbox.getY() * 0.8;

    // m_container.m_Collection.manualMove(speed);
    m_container.m_Collection.
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
