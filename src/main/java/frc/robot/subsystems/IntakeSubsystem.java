// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  // initalize motor
  private final TalonFX intakeMotor;

  // set control type
  private final VelocityVoltage velocityRequest;

  public IntakeSubsystem() {
    // Change CAN ID FOR TESTING
    intakeMotor = new TalonFX(51);

    // Configure motor
    intakeMotor.getConfigurator().apply(Configs.intakeMotor.intakeConfig);

    // Setup velocity
    velocityRequest = new VelocityVoltage(0).withSlot(0);
  }

  public void start() {
    intakeMotor.setControl(velocityRequest.withVelocity(100)); // make sure this is double
  }

  public void stop() {
    intakeMotor.stopMotor();
  }

  public boolean atTargetSpeed(double targetRPS, double tolerance) {
      return Math.abs(intakeMotor.getVelocity().getValueAsDouble() - targetRPS) < tolerance;
  }

  @Override
  public void periodic() {
  }
}