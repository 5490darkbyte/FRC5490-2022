// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShootWithTimout extends CommandBase {

  RobotContainer m_Container;
  double timout;

  Timer cutoffTimer = new Timer();

  boolean started = false;

  /** Creates a new ShootWithTimout. */
  public ShootWithTimout(RobotContainer container, double timeout) {
    this.timout = timeout;
    this.m_Container = container;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!started) {
      cutoffTimer.reset();
      cutoffTimer.start();
      started = true;
     }

    m_Container.m_intakeOuttake.dump();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Container.m_intakeOuttake.stop();
    started = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cutoffTimer.hasElapsed(timout);
  }
}
