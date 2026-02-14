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

  // Left and right leader motors - all PID and velocity commands go here
  private final TalonFX leftLeaderMotor;
  private final TalonFX rightLeaderMotor;
  
  // Left and right follower motors - mirrors the leader with opposite direction
  private final TalonFX leftFollowerMotor;
  private final TalonFX rightFollowerMotor;

  private final VelocityVoltage velocityRequest;

  private final Timer atSpeedTimer = new Timer();
  private static final double REQUIRED_TIME_AT_SPEED = 0.2;

  private double targetRPS = 0;

  public ShooterSubsystem() {
      leftLeaderMotor = new TalonFX(52);
      leftFollowerMotor = new TalonFX(40);
      rightLeaderMotor = new TalonFX(0);
      rightFollowerMotor = new TalonFX(0);

      velocityRequest = new VelocityVoltage(0).withSlot(0);

      leftLeaderMotor.getConfigurator().apply(Configs.shooterMotor.leftShooterConfig);
      leftFollowerMotor.getConfigurator().apply(Configs.shooterMotor.leftShooterConfig); 
      rightLeaderMotor.getConfigurator().apply(Configs.shooterMotor.rightShooterConfig);
      rightFollowerMotor.getConfigurator().apply(Configs.shooterMotor.rightShooterConfig);

      // Set follower relationship
      leftFollowerMotor.setControl(
        new Follower(leftLeaderMotor.getDeviceID(), MotorAlignmentValue.Aligned)
      );

      rightFollowerMotor.setControl(
        new Follower(rightLeaderMotor.getDeviceID(), MotorAlignmentValue.Aligned)
      );
  }

  /**
   * Set shooter velocity in rotations per second
   * @param rps Target velocity in RPS
   */
  public void setVelocity(double rps) {
    targetRPS = rps;
    leftLeaderMotor.setControl(velocityRequest.withVelocity(rps));
    rightLeaderMotor.setControl(velocityRequest.withVelocity(rps));
  }

  /** Stop both pair of motors */
  public void stop() {
    targetRPS = 0;
    atSpeedTimer.stop();
    atSpeedTimer.reset();
    
    leftLeaderMotor.stopMotor();
    rightLeaderMotor.stopMotor();
  }

  /** Get left (leader) motor velocity in RPS */
  public double getLeftSpeed() {
    return leftLeaderMotor.getVelocity().getValueAsDouble();
  }

  /** Get right (leader) motor velocity in RPS */
  public double getRightSpeed() {
    return rightLeaderMotor.getVelocity().getValueAsDouble();
  }

  /**
   * Check if shooter is at target speed for required duration
   * @param tolerance Allowed error in RPS
   * @return True if at speed for REQUIRED_TIME_AT_SPEED seconds
   */
  public boolean atTargetSpeed(double tolerance) {
    // Only check leader motor since follower mirrors it
    boolean withinTolerance = (Math.abs(getLeftSpeed() - targetRPS) < tolerance) && (Math.abs(getRightSpeed() - targetRPS) < tolerance);

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