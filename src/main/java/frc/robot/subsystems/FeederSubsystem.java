// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class FeederSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */

  // initalize motor
  private final TalonFX feederMotor;

  // set control type
  private final VelocityVoltage velocityRequest;

  public FeederSubsystem() {
    // Change CAN ID FOR TESTING
    feederMotor = new TalonFX(31);

    // Configure motor
    feederMotor.getConfigurator().apply(Configs.feederMotor.feederConfig);

    // Setup velocity
    velocityRequest = new VelocityVoltage(0).withSlot(0);
  }

  public void start() {
    feederMotor.setControl(velocityRequest.withVelocity(100)); // make sure this is double
  }

  public void stop() {
    feederMotor.stopMotor();
  }

  public boolean atTargetSpeed(double targetRPS, double tolerance) {
      return Math.abs(feederMotor.getVelocity().getValueAsDouble() - targetRPS) < tolerance;
  }

  @Override
  public void periodic() {
  }
}