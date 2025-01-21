// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autons;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.HomePositionCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RevUpCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.constants.ShooterConstants;
import org.frcteam3539.Byte_Swerve_Lib.io.BBMPLoader;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueShootLeft extends SequentialCommandGroup {
	BBMPLoader loader = new BBMPLoader("/home/lvuser/profiles/BlueShootLeft.txt", false);

	private Command[] sequence = {
			new InstantCommand(() -> RobotContainer.drivetrainSubsystem.seedFieldRelative(loader.getFirstTrajectory())),

			new ParallelCommandGroup(new RevUpCommand(false, ShooterConstants.shootDps).withTimeout(10),

					new SequentialCommandGroup(new WaitCommand(1), new ShootCommand().withTimeout(1)),
					new SequentialCommandGroup(new WaitCommand(5), new AutoShootCommand().withTimeout(4)),
					new SequentialCommandGroup(new WaitCommand(7), new ShootCommand().withTimeout(1)),
					new SequentialCommandGroup(new WaitCommand(10), new HomePositionCommand()),

					new SequentialCommandGroup(new WaitCommand(1.25),
							new IntakeCommand(true, IntakeMode.FRONT).withTimeout(2.5)),

					new SequentialCommandGroup(new WaitCommand(1),
							new FollowTrajectoryCommand(RobotContainer.drivetrainSubsystem, loader.getNextTrajectory()),
							new WaitCommand(4),
							new FollowTrajectoryCommand(RobotContainer.drivetrainSubsystem, loader.getNextTrajectory())

					))};

	/** Creates a new RedShootDrive. */
	public BlueShootLeft() {
		addCommands(sequence);
	}
}
