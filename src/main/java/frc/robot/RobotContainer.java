// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Collection;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Dump;
import frc.robot.commands.Intake;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.ManualMoveCollectorArm;
import frc.robot.commands.TogglePIDSetPoint;
import frc.robot.commands.auto.AutoLineDrive;
import frc.robot.commands.auto.ShootWithTimout;
import frc.robot.subsystems.CollectorArm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeOuttake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public final Drivetrain m_drivetrain = new Drivetrain();
	public final CollectorArm m_Collection = new CollectorArm(this);
	public final IntakeOuttake m_intakeOuttake = new IntakeOuttake();

	private Command m_tempAutoCommand;
	private ParallelCommandGroup m_manualCommand = new ParallelCommandGroup(new ManualDrive(this), new ManualMoveCollectorArm(this));

	public Joystick joystick = new Joystick(0);
	public Joystick xbox = new Joystick(1);

	public JoystickButton trigger;
	public JoystickButton thumb;

	public JoystickButton leftButton;
	public JoystickButton rightButton;

	// private JoystickButton leftTrigger;
	// private Trigger rightButton;
	public JoystickButton aButton;
	public JoystickButton bButton;
	public JoystickButton yButton;
	public JoystickButton xButton;
	// Top buttons (counterclockwise from top left: 5-3-4-6)
	public JoystickButton button5;
	public JoystickButton button3;
	public JoystickButton liftManualDown;
	public JoystickButton liftManualUp;

	// Side buttons (from top left)
	public JoystickButton button4;
	public JoystickButton button6;
	public JoystickButton button7;
	public JoystickButton button8;
	public JoystickButton button9;
	public JoystickButton button10;
	public JoystickButton button11;
	public JoystickButton button12;

	public int xboxLeftTrigger = 2;
	public int xboxRightTrigger = 3;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		// BUTTON MAPPINGS
		trigger = new JoystickButton(joystick, 1);
		thumb = new JoystickButton(joystick, 2);

		// Top buttons (counterclockwise from top left: 5-3-4-6)
		button5 = new JoystickButton(joystick, 5);
		button3 = new JoystickButton(joystick, 3);
		button4 = new JoystickButton(joystick, 4);
		button6 = new JoystickButton(joystick, 6);

		// Side buttons (from top left)

		button7 = new JoystickButton(joystick, 7);
		button8 = new JoystickButton(joystick, 8);
		button9 = new JoystickButton(joystick, 9);
		button10 = new JoystickButton(joystick, 10);
		button11 = new JoystickButton(joystick, 11);
		button12 = new JoystickButton(joystick, 12);
		aButton = new JoystickButton(xbox, 1);
		bButton = new JoystickButton(xbox, 2);
		xButton = new JoystickButton(xbox, 3);
		yButton = new JoystickButton(xbox, 4);

		leftButton = new JoystickButton(xbox, 5);
		rightButton = new JoystickButton(xbox, 6);

		// Configure the button bindings
		configureButtonBindings();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
	 * it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		// aButton.when

		//for collector arm - xbox left stick will move set point

		// for intakeOutake, one xbox trigger will enable dumping when pressed and the associated bumper butotn wll toggle on and off holding/collecting mode
		
		aButton.whenHeld(new Dump(this));
		bButton.whenHeld(new Intake(this));

		xButton.whenReleased(new TogglePIDSetPoint(this));

		yButton.whenReleased(new ShootWithTimout(this, 3));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {


		//KREMOVE test distances

		// if (m_tempAutoCommand == null) {
			//m_tempAutoCommand = new AutoLineDrive(this,-2);
			m_tempAutoCommand = new SequentialCommandGroup(
				new AutoLineDrive(this, 1.3),
				//dump commandfor about 3s
				new ShootWithTimout(this, 2),
				new AutoLineDrive(this, -2)
			);
		// }

		// An ExampleCommand will run in autonomous
		return m_tempAutoCommand;
	}

	public Command getTeleopCommand() {
		// An ExampleCommand will run in autonomous
		return m_manualCommand;
	}
}
