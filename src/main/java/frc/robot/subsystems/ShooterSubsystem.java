package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX leftShooterMotor;
  private final TalonFX rightShooterMotor;

  private final VelocityVoltage velocityRequest;
  
  private final Timer atSpeedTimer = new Timer();
  private static final double REQUIRED_TIME_AT_SPEED = 0.2; // seconds


  private double targetRPS = 0;

  public ShooterSubsystem() {
    leftShooterMotor = new TalonFX(52);
    rightShooterMotor = new TalonFX(40);

    velocityRequest = new VelocityVoltage(0).withSlot(0);

    leftShooterMotor.getConfigurator().apply(Configs.shootingMotor.shootingLeftConfig);
    rightShooterMotor.getConfigurator().apply(Configs.shootingMotor.shootingRightConfig);

    // If needed:
    // rightShooterMotor.setInverted(true);
  }

  /** Set both motors to ANY velocity (RPS) */
  public void setVelocity(double rps) {
    targetRPS = rps;

    leftShooterMotor.setControl(velocityRequest.withVelocity(rps));
    rightShooterMotor.setControl(velocityRequest.withVelocity(rps));
  }

  /** Optional: allow independent wheel tuning */
  public void setVelocity(double leftRPS, double rightRPS) {
    targetRPS = leftRPS; // primary reference (for atTargetSpeed)

    atSpeedTimer.stop();
    atSpeedTimer.reset();

    leftShooterMotor.setControl(velocityRequest.withVelocity(leftRPS));
    rightShooterMotor.setControl(velocityRequest.withVelocity(rightRPS));
  }


  /** Stop both motors */
  public void stop() {
    leftShooterMotor.stopMotor();
    rightShooterMotor.stopMotor();
    targetRPS = 0;
    atSpeedTimer.stop();
    atSpeedTimer.reset();
  }

  public double getLeftSpeed() {
    return leftShooterMotor.getVelocity().getValueAsDouble();
  }

  public double getRightSpeed() {
    return rightShooterMotor.getVelocity().getValueAsDouble();
  }

  /** Both motors must be within tolerance of target */
  public boolean atTargetSpeed(double tolerance) {
  boolean withinTolerance =
      Math.abs(getLeftSpeed() - targetRPS) < tolerance &&
      Math.abs(getRightSpeed() - targetRPS) < tolerance;

  if (withinTolerance) {
    if (!atSpeedTimer.isRunning()) {
      atSpeedTimer.restart(); // start counting once we first enter range
    }
  } else {
    atSpeedTimer.stop();
    atSpeedTimer.reset();
  }

  return atSpeedTimer.hasElapsed(REQUIRED_TIME_AT_SPEED);
}


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Target RPS", targetRPS);
    SmartDashboard.putNumber("Shooter Left RPS", getLeftSpeed());
    SmartDashboard.putNumber("Shooter Right RPS", getRightSpeed());
  }
}