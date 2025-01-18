// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;

public class ClimberSubsystem extends SubsystemBase {
	// Declare
	private TalonFX rightclimbMotor, leftclimbMotor;
	public ClimberSubsystem() {
		// Initialize

		// Initialize
		rightclimbMotor = new TalonFX(IDConstants.rightClimbMotorID, "rio");
		rightclimbMotor.getConfigurator().apply(new TalonFXConfiguration());
		rightclimbMotor.setInverted(false);

		leftclimbMotor = new TalonFX(IDConstants.leftClimbMotorID, "rio");
		leftclimbMotor.getConfigurator().apply(new TalonFXConfiguration());
		leftclimbMotor.setInverted(false);

	}

	// Get/set

	public void setrightclimbMotorVoltage(double voltage) {
		rightclimbMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
	}

	public void setleftclimbMotorVoltage(double voltage) {
		leftclimbMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
	}

	public void log() {
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
