// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;

public class ShooterSubsystem extends SubsystemBase {
	// Declare
	private TalonFX topMotor, bottomMotor, feedMotor, elevatorMotor, angleMotor;

	public ShooterSubsystem() {
		topMotor = new TalonFX(IDConstants.topMotor, "rio");
		topMotor.getConfigurator().apply(new TalonFXConfiguration());
		topMotor.setInverted(false);
		bottomMotor = new TalonFX(IDConstants.bottomMotor, "rio");
		bottomMotor.getConfigurator().apply(new TalonFXConfiguration());
		bottomMotor.setInverted(false);
		feedMotor = new TalonFX(IDConstants.feedMotor, "rio");
		feedMotor.getConfigurator().apply(new TalonFXConfiguration());
		feedMotor.setInverted(false);

		// Initialize
	}

	// Get/set

	public void log() {
	}

	@Override
	public void periodic() {
	}

	public void setTopMotorVoltage(double voltage) {
		topMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
	}

	public void setBottomMotorVoltage(double voltage) {
		bottomMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
	}
	public void setFeedMotorVoltage(double voltage) {
		feedMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
	}
}
