package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class IntakeConstants extends BBConstants {
	public IntakeConstants() {
		super("/home/lvuser/IntakeConstants.ini", true);
		save();
	}

	public static double intakeDps = 200.0;
	public static double chamberV = 0.125;
	public static double chamberP = 0.25;
	public static double groundV = 0.125;
	public static double groundP = 0.25;
	public static double kickP = 0.25;
	public static double kickV = 0.125;
	public static double groundWheelDiameter = 2.125;
	public static double kickWheelDiameter = 1.625;
	public static double chamberWheelDiameter = 1.625;
	public static double kickGearRatio = 1.2;
	public static double intakeShutOffDelay = 0.125;
	public static int invertSensors = 1;
}