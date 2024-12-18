package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class ClimberConstants extends BBConstants {
	public ClimberConstants() {
		super("/home/lvuser/ClimberConstants.ini", true);
		save();
	}

	public static double buddyClimbRps = 0.0;
	public static double buddyClimbThreshold = 10.0;
	public static double maxClimbRps = 0.0;
}