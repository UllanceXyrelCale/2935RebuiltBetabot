
package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor;
  private final VelocityVoltage velocityRequest;

  public IntakeSubsystem() {
    intakeMotor = new TalonFX(33);
    velocityRequest = new VelocityVoltage(0).withSlot(0);

    intakeMotor.getConfigurator().apply(Configs.intakeMotor.intakeConfig);
  }

  public void setVelocity(double rps) {
    intakeMotor.setControl(velocityRequest.withVelocity(rps)); 
  }

  public void stop() {
    intakeMotor.stopMotor();
  }

  public double getIntakeSpeed() {
    return intakeMotor.getVelocity().getValueAsDouble();
  }

  public boolean atTargetSpeed(double targetRPS, double tolerance) {
      return Math.abs(getIntakeSpeed() - targetRPS) < tolerance;
  }

  @Override
  public void periodic() {
  }
}