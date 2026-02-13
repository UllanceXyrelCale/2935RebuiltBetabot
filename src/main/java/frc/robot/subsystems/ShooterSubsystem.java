package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class ShooterSubsystem extends SubsystemBase {

  // Left motor is the leader - all PID and velocity commands go here
  private final TalonFX leftShooterMotor;
  // Right motor is the follower - mirrors the leader with opposite direction
  private final TalonFX rightShooterMotor;

  private final VelocityVoltage velocityRequest;

  private final Timer atSpeedTimer = new Timer();
  private static final double REQUIRED_TIME_AT_SPEED = 0.2;

  private double targetRPS = 0;

  public ShooterSubsystem() {
      leftShooterMotor = new TalonFX(52);
      rightShooterMotor = new TalonFX(40);

      velocityRequest = new VelocityVoltage(0).withSlot(0);

      // ONLY apply config to the LEADER
      leftShooterMotor.getConfigurator().apply(Configs.shooterMotor.shooterConfig);
      
      // For follower: either apply minimal config or just factory default
      rightShooterMotor.getConfigurator().apply(Configs.shooterMotor.shooterConfig); // Factory defaults
      
      // Now set follower relationship
      rightShooterMotor.setControl(
        new Follower(leftShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed)
      );
  }

  /**
   * Set shooter velocity in rotations per second
   * @param rps Target velocity in RPS
   */
  public void setVelocity(double rps) {
    targetRPS = rps;
    leftShooterMotor.setControl(velocityRequest.withVelocity(rps));
    // Right motor follows automatically
  }

  /** Stop both motors */
  public void stop() {
    targetRPS = 0;
    atSpeedTimer.stop();
    atSpeedTimer.reset();
    leftShooterMotor.stopMotor();
    // Follower automatically stops with leader
  }

  /** Get left (leader) motor velocity in RPS */
  public double getLeftSpeed() {
    return leftShooterMotor.getVelocity().getValueAsDouble();
  }

  /** Get right (follower) motor velocity in RPS (will be opposite sign) */
  public double getRightSpeed() {
    return rightShooterMotor.getVelocity().getValueAsDouble();
  }

  /**
   * Check if shooter is at target speed for required duration
   * @param tolerance Allowed error in RPS
   * @return True if at speed for REQUIRED_TIME_AT_SPEED seconds
   */
  public boolean atTargetSpeed(double tolerance) {
    // Only check leader motor since follower mirrors it
    boolean withinTolerance = Math.abs(getLeftSpeed() - targetRPS) < tolerance;

    if (withinTolerance) {
      if (!atSpeedTimer.isRunning()) {
        atSpeedTimer.restart();
      }
    } else {
      atSpeedTimer.stop();
      atSpeedTimer.reset();
    }

    return atSpeedTimer.hasElapsed(REQUIRED_TIME_AT_SPEED);
  }

  /** Get current target velocity in RPS */
  public double getTargetRPS() {
    return targetRPS;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Target RPS", targetRPS);
    SmartDashboard.putNumber("Shooter/Left RPS", getLeftSpeed());
    SmartDashboard.putNumber("Shooter/Right RPS", getRightSpeed());
    SmartDashboard.putBoolean("Shooter/At Speed", atTargetSpeed(2.0)); // Example tolerance
  }
}