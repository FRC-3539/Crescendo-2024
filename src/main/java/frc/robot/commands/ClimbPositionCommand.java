// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command hinline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbPositionCommand extends SequentialCommandGroup {
	private Command[] commands = {
			new ParallelCommandGroup(new AngleShooterCommand(71), new SetElevatorCommand(6), new ReleaseStabbyFeet())};

	/** Creates a new AmpCommand. */
	public ClimbPositionCommand() {

		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(commands);
	}

}
