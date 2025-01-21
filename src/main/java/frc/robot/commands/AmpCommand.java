// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ShooterConstants;

// NOTE:  Consider using this command hinline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpCommand extends SequentialCommandGroup {
	private Command[] commands = {new ParallelCommandGroup(new AngleShooterCommand(34), new SetElevatorCommand(2.5),
			new RevUpCommand(false, ShooterConstants.ampDps))};

	/** Creates a new AmpCommand. */
	public AmpCommand() {

		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		addCommands(commands);
	}

}
