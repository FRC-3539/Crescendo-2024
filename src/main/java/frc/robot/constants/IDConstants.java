package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class IDConstants extends BBConstants {
	public IDConstants() {
		super("/home/lvuser/IDConstants.ini", true);
		save();
	}

	public static int topMotor = 15;
	public static int bottomMotor = 14;
	public static int feedMotor = 16;
	public static int groundMotorID = 8;
	public static int kickMotorID = 9;
	public static int grabMotorID = 17;
	public static int BRDriveID = 0;
	public static int BRCanCoderID = 30;
	public static int BRSteeringID = 1;
	public static int BLDriveID = 4;
	public static int BLCanCoderID = 33;
	public static int BLSteeringID = 5;
	public static int FRDriveID = 2;
	public static int FRCanCoderID = 31;
	public static int FRSteeringID = 3;
	public static int FLDriveID = 6;
	public static int FLCanCoderID = 32;
	public static int FLSteeringID = 7;
	public static int rightClimbMotorID = 10;
	public static int leftClimbMotorID = 11;
	public static int PigeonID = 41;
	public static int CANdleID = 40;
	public static String CandleCanName = "canivore";
	public static String swerveCanbusName = "canivore";
	public static int buddyClimbMotorID = 18;
	public static int angleCanCoderID = 34;
	public static int angleMotorID = 13;
	public static int elevatorMotorID = 12;
	public static int shooterSensorChannel = 3;
	public static int frontSensorChannel = 2;
	public static int backSensorChannel = 0;
	public static int chamberSensorChannel = 1;
	public static int leftServoChannel = 1;
	public static int rightServoChannel = 0;
}