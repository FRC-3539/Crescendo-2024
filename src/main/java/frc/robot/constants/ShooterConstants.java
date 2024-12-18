package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class ShooterConstants extends BBConstants {
	public ShooterConstants() {
		super("/home/lvuser/ShooterConstants.ini", true);
		save();
	}

	public static double shootDps = 1000.0;
	public static double feedDps = 1000.0;
	public static double elevatorDps = 0.0;
	public static double angleShooterP = 120.0;
	public static double angleShooterI = 0.0;
	public static double angleShooterD = 0.0;
	public static double angleShooterV = 12.0;
	public static double angleShooterG = 0.29;
	public static double angleShooterSoftMin = -0.118;
	public static double angleShooterSoftMax = 0.35;
	public static double elevatorMotorP = 12.0;
	public static double elevatorMotorI = 0.0;
	public static double elevatorMotorD = 0.0;
	public static double elevatorMotorV = 0.125;
	public static double elevatorMotorG = 0.348;
	public static double elevatorSoftMin = 0.0;
	public static double elevatorSoftMax = 50.0;
	public static double angleMotorToEncoder = 81.0;
	public static double elevatorMotorToInches = 0.1598888;
	public static double shooterAngleOffset = -0.57283;
	public static double minElevatorMoveAngle = 0.0;
	public static double shootP = 0.25;
	public static double shootV = 0.125;
	public static double shootWheelDiameter = 4.0;
	public static double feedWheelDiameter = 4.0;
	public static double feedP = 0.25;
	public static double feedV = 0.125;
	public static double restShooterAngle = -55.0;
	public static double elevatorCollisionHeight = 5.0;
	public static double ampDps = 250.0;
	public static double shooterRestingRotations = -0.118;
	public static int UseForwardLimitForReverse = 1;
}