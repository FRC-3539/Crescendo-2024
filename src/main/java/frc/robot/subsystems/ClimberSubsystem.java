// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;

public class ClimberSubsystem extends SubsystemBase {
	private TalonFX leftClimbMotor, rightClimbMotor;
	private Servo leftServo, rightServo;
	private double leftServoPosition = 1;
	private double rightServoPosition = 0;

	public ClimberSubsystem() {
		MotorOutputConfigs rightOutputConfig = new MotorOutputConfigs();
		rightOutputConfig.Inverted = InvertedValue.Clockwise_Positive;
		// rightOutputConfig.Inverted = InvertedValue.Clockwise_Positive;

		MotorOutputConfigs leftOutputConfig = new MotorOutputConfigs();
		leftOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
		// leftOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;

		MotorOutputConfigs buddyOutputConfig = new MotorOutputConfigs();
		buddyOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;

		leftClimbMotor = new TalonFX(IDConstants.leftClimbMotorID, "rio");
		leftClimbMotor.getConfigurator().apply(leftOutputConfig);
		rightClimbMotor = new TalonFX(IDConstants.rightClimbMotorID, "rio");
		rightClimbMotor.getConfigurator().apply(rightOutputConfig);

		leftServo = new Servo(IDConstants.leftServoChannel);
		rightServo = new Servo(IDConstants.rightServoChannel);

		// buddyClimbMotor = new TalonFX(IDConstants.buddyClimbMotorID, "rio");
		// buddyClimbMotor.getConfigurator().apply(buddyOutputConfig);

		leftClimbMotor.setNeutralMode(NeutralModeValue.Brake);
		rightClimbMotor.setNeutralMode(NeutralModeValue.Brake);

		leftClimbMotor.getConfigurator().apply(new HardwareLimitSwitchConfigs().withForwardLimitEnable(true));
		rightClimbMotor.getConfigurator().apply(new HardwareLimitSwitchConfigs().withForwardLimitEnable(true));

	}

	public void setLeftClimbMotorSpeed(double rps) {
		leftClimbMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
	}

	public void setLeftClimbMotorVoltage(double voltage) {
		leftClimbMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
	}

	public void setRightClimbMotorSpeed(double rps) {
		rightClimbMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
	}

	public void setRightClimbMotorVoltage(double voltage) {
		rightClimbMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
	}

	public void setLeftServoPosition(double position) {
		leftServoPosition = position;
	}
	public void setRightServoPosition(double position) {
		rightServoPosition = position;
	}
	// public void setBuddyClimbMotorSpeed(double rps) {
	// buddyClimbMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
	// }

	// public void setBuddyClimbMotorVoltage(double voltage) {
	// buddyClimbMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
	// }

	// public double getBuddyClimbMotorSpeed() {

	// return buddyClimbMotor.getVelocity().getValue();
	// }
	public void setClimberBreakMode(boolean enabled) {
		if (enabled) {
			leftClimbMotor.setNeutralMode(NeutralModeValue.Brake);
			rightClimbMotor.setNeutralMode(NeutralModeValue.Brake);
		} else {
			leftClimbMotor.setNeutralMode(NeutralModeValue.Coast);
			rightClimbMotor.setNeutralMode(NeutralModeValue.Coast);
		}
	}
	public boolean doneClimbing() {
		return leftClimbMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround
				&& rightClimbMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;

	}

	public void log() {
	}

	@Override
	public void periodic() {
		leftServo.set(leftServoPosition);
		rightServo.set(rightServoPosition);
		// This method will be called once per scheduler run
	}
}
