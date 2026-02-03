package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  // Initialize variables needed to run subsystem
  private final TalonFX leftMotor, rightMotor;
  private final List <TalonFX> motors;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  public ShooterSubsystem() {
    leftMotor = new TalonFX(41);
    rightMotor = new TalonFX(42);

    configureMotor(leftMotor, InvertedValue.Clockwise_Positive);
    configureMotor(rightMotor, InvertedValue.CounterClockwise_Positive);

    motors = List.of(leftMotor, rightMotor);
  }

  public void configureMotor(TalonFX motor, InvertedValue invertDirection) {
    // ----------- Configures the motor ---------- //
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Set motor outputs
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = invertDirection;

    // Current limit
    config.CurrentLimits.SupplyCurrentLimit = 0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Feedback
    config.Feedback.SensorToMechanismRatio = 0;

    // Set up PID
    config.Slot0.kP = 0;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.Slot0.kV = 0;

    // Set up voltage 
    config.Voltage.PeakForwardVoltage = 12;
    config.Voltage.PeakReverseVoltage = 12;

    // Applies the actual configs to the motor
    motor.getConfigurator().apply(config); 
  }

  public boolean atTargetSpeed() {
     return motors.stream().allMatch(motor -> {
            final boolean isInVelocityMode = motor.getAppliedControl().equals(velocityRequest);
            final AngularVelocity currentVelocity = motor.getVelocity().getValue();
            final AngularVelocity targetVelocity = velocityRequest.getVelocityMeasure();
            return isInVelocityMode && currentVelocity.isNear(targetVelocity, 50);
        });
  }

  public void setShooter(double rpm) {
    for (TalonFX motor : motors) {
      motor.setControl(velocityRequest.withVelocity(RPM.of(rpm)));
    }
  }

  public Command shooterCommand(double rpm) {
    return runOnce(() -> setShooter(rpm))
    .andThen(Commands.waitUntil(this::atTargetSpeed));
  }

  public Command shooterStop() {
    return runOnce(() -> setShooter(0));
  }

  @Override
  public void periodic() {
  }
}
