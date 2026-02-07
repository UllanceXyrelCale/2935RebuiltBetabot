// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  // initalize motor
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  // set control type
  private final VelocityVoltage velocityRequest;

  public ShooterSubsystem() {
    // Change CAN ID FOR TESTING
    leftMotor = new TalonFX(52);
    rightMotor = new TalonFX(40);

    // Configure motor
    leftMotor.getConfigurator().apply(Configs.shootingMotor.shootingConfig);
    rightMotor.getConfigurator().apply(Configs.shootingMotor.shootingConfig);

    // Setup velocity
    velocityRequest = new VelocityVoltage(0).withSlot(0);
  }

  public void setVelocity(double shooterVelocity) {
    leftMotor.setControl(velocityRequest.withVelocity(shooterVelocity)); // make sure this is double
    rightMotor.setControl(velocityRequest.withVelocity(shooterVelocity));
  }

  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public boolean atTargetSpeed(double targetRPS, double tolerance) {
      return Math.abs(leftMotor.getVelocity().getValueAsDouble() - targetRPS) < tolerance && Math.abs(rightMotor.getVelocity().getValueAsDouble() - targetRPS) < tolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Velocity", leftMotor.getVelocity().getValueAsDouble());
  }
}