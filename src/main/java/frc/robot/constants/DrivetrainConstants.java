package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class DrivetrainConstants extends BBConstants {
	public DrivetrainConstants() {
		super("/home/lvuser/DrivetrainConstants.ini", true);
		save();
	}

	public static double TranslationkP = 20.0;
	public static double TranslationkI = 0.0;
	public static double TranslationkD = 0.0;
	public static double RotationkP = 15.0;
	public static double RotationkI = 0.0;
	public static double RotationkD = 0.0;
	public static double TranslationkV = 1.0;
	public static double TranslationkA = 0.0;
	public static double TranslationkS = 0.0;
	public static double FLSteerOffset = 0.0;
	public static double FRSteerOffset = 0.0;
	public static double BLSteerOffset = 0.0;
	public static double BRSteerOffset = 0.0;
	public static double speedMultiplier = 1.0;
	public static double rotationSpeedMultiplier = 0.5;
	public static double turboSpeedMultiplier = 0.4;
	public static double turboRotationSpeedMultiplier = 0.3;
	public static double AlignkP = 2.0;
}