// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class AutoLineDrive extends CommandBase {


  RobotContainer m_container;
  Drivetrain m_drivetrain;

  //in meters
  double distance;

  /** Creates a new AutoLineDrive. distance is in meters*/
  public AutoLineDrive(RobotContainer container,double distance) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.distance = distance;
    this.m_drivetrain = container.m_drivetrain;
    this.m_container = container;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(container.m_drivetrain); 

    

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //set up pid
    m_drivetrain.resetPID();
    m_drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.pidDrive(distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //check if within range of distance -- in future ad buffer time
  }
}
