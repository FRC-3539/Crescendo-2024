// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;
import frc.robot.constants.LedConstants;

public class LedSubsystem extends SubsystemBase {

	static boolean intaking;
	static boolean enabled;
	static boolean aligning;
	static boolean autoShooting;
	static boolean shootAligning;
	static boolean noteTracking;
	static boolean climbing;
	static boolean reverseClimbing;
	static CANdle candle;

	public LedSubsystem(boolean enable) {
		enabled = enable;

		candle = new CANdle(IDConstants.CANdleID, IDConstants.CandleCanName);
		candle.configLEDType(LEDStripType.GRB);
		candle.configBrightnessScalar(LedConstants.maxBrightness);
		setLEDs(LEDState.ON);
	}

	public enum LEDState {
		ON, OFF, CONNECTED, READY, INTAKING, INTAKING_EMPTY, SHOOTING, PREPARED, CLIMBING, REVERSE_CLIMBING, AUTO, ERROR, FRONT, BACK
	}

	public static LEDState state;

	public static void setLEDs(LEDState state) {
		if (!enabled)
			return;

		switch (state) {
			case OFF :
				candle.animate(null);
				candle.setLEDs(0, 0, 0);
				break;
			case ON :
				candle.animate(new ColorFlowAnimation(LedConstants.Green.getRed(), LedConstants.Green.getGreen(),
						LedConstants.Green.getBlue(), 0, LedConstants.flashSpeed, LedConstants.numLights,
						Direction.Forward));

				break;
			case CONNECTED :
				candle.animate(null);
				candle.setLEDs(LedConstants.Green.getRed(), LedConstants.Green.getGreen(),
						LedConstants.Green.getBlue());
				break;

			case READY :
				candle.animate(new LarsonAnimation(LedConstants.Green.getRed(), LedConstants.Green.getGreen(),
						LedConstants.Green.getBlue(), 0, 0.5, LedConstants.numLights, BounceMode.Back, 6));
				break;
			case INTAKING :
				candle.animate(new StrobeAnimation(LedConstants.Orange.getRed(), LedConstants.Orange.getGreen(),
						LedConstants.Orange.getBlue(), 0, LedConstants.flashSpeed, LedConstants.numLights));
				break;
			case INTAKING_EMPTY :
				candle.animate(new StrobeAnimation(0, 255, 0, 0, LedConstants.flashSpeed, LedConstants.numLights));
				break;

			case SHOOTING :
				candle.animate(new ColorFlowAnimation(LedConstants.Orange.getRed(), LedConstants.Orange.getGreen(),
						LedConstants.Orange.getBlue(), 0, LedConstants.flashSpeed, LedConstants.numLights, null));
				break;

			case PREPARED :
				candle.animate(null);
				candle.setLEDs(LedConstants.Orange.getRed(), LedConstants.Orange.getGreen(),
						LedConstants.Orange.getBlue(), 0, 0, LedConstants.numLights);

				break;

			case CLIMBING :
				candle.animate(new RainbowAnimation(LedConstants.maxBrightness, 1, LedConstants.numLights));
				break;
			case REVERSE_CLIMBING :
				candle.animate(new FireAnimation(LedConstants.maxBrightness, LedConstants.flashSpeed,
						LedConstants.numLights, 0, 1));
				break;
			case AUTO :
				candle.animate(new StrobeAnimation(LedConstants.Blue.getRed(), LedConstants.Blue.getGreen(),
						LedConstants.Blue.getBlue(), 0, LedConstants.flashSpeed, LedConstants.numLights));
				break;
			case ERROR :
				candle.animate(null);
				candle.setLEDs(255, 0, 0, 0, 0, LedConstants.numLights);
				break;
			case FRONT :
				candle.animate(new StrobeAnimation(LedConstants.Yellow.getRed(), LedConstants.Yellow.getGreen(),
						LedConstants.Yellow.getBlue(), 0, LedConstants.flashSpeed, 34));
				candle.setLEDs(LedConstants.Yellow.getRed(), LedConstants.Yellow.getGreen(),
						LedConstants.Yellow.getBlue(), 0, 34, 12);
				break;
			case BACK :
				candle.animate(new StrobeAnimation(LedConstants.Yellow.getRed(), LedConstants.Yellow.getGreen(),
						LedConstants.Yellow.getBlue(), 0, LedConstants.flashSpeed, LedConstants.numLights, 34));
				candle.setLEDs(LedConstants.Yellow.getRed(), LedConstants.Yellow.getGreen(),
						LedConstants.Yellow.getBlue(), 0, 0, 34);
				break;

			default :
				break;
		}
	}
	public static void setIntaking(boolean intake) {
		intaking = intake;
	}
	public static void setAligning(boolean align) {
		aligning = align;
	}
	public static void setAutoShooting(boolean autoShoot) {
		autoShooting = autoShoot;
	}
	public static void setShootAligning(boolean shootAlign) {
		shootAligning = shootAlign;
	}
	public static void setNoteTracking(boolean noteTrack) {
		noteTracking = noteTrack;
	}
	public static void setClimbing(boolean climb) {
		climbing = climb;
	}
	public static void setReverseClimbing(boolean reverseClimb) {
		reverseClimbing = reverseClimb;
	}

	@Override
	public void periodic() {
		if (DriverStation.isAutonomous() && !VisionSubsystem.backLeftCam.isConnected()
				&& !VisionSubsystem.backRightCam.isConnected()) {
			setLEDs(LEDState.ERROR);
			return;
		}
		if (autoShooting && !VisionSubsystem.backLeftCam.isConnected() && !VisionSubsystem.backRightCam.isConnected()) {
			setLEDs(LEDState.ERROR);
			return;
		}
		if (shootAligning && !VisionSubsystem.backLeftCam.isConnected()
				&& !VisionSubsystem.backRightCam.isConnected()) {
			setLEDs(LEDState.ERROR);
			return;
		}
		if (aligning) {
			setLEDs(LEDState.AUTO);
			return;
		}

		if (shootAligning) {
			setLEDs(LEDState.AUTO);
			return;
		}
		if (ClimberSubsystem.doneClimbing()) {
			setLEDs(LEDState.CLIMBING);
			return;
		}
		if (reverseClimbing) {
			setLEDs(LEDState.REVERSE_CLIMBING);
			return;
		}
		if (noteTracking) {
			if (!VisionSubsystem.backNoteCam.isConnected() || !VisionSubsystem.frontNoteCam.isConnected()) {
				setLEDs(LEDState.ERROR);
				return;
			}
			setLEDs(LEDState.AUTO);
			return;
		}
		// This method will be called once per scheduler run
		// setLEDs(LEDState.READY);
		if (ShooterSubsystem.getShooterSensor()) {
			setLEDs(LEDState.PREPARED);
			return;
		} else {
			if (IntakeSubsystem.getChamberSensor()) {
				setLEDs(LEDState.INTAKING);
				return;
			}
		}
		if (IntakeSubsystem.getFrontSensor()) {

			setLEDs(LEDState.FRONT);
			return;

		} else if (IntakeSubsystem.getBackSensor()) {

			setLEDs(LEDState.BACK);
			return;
		}
		if (intaking) {
			if (IntakeSubsystem.getChamberSensor()) {

				setLEDs(LEDState.INTAKING);
				return;

			} else {
				setLEDs(LEDState.INTAKING_EMPTY);
				return;
			}
		}

		setLEDs(LEDState.CONNECTED);
	}
}
