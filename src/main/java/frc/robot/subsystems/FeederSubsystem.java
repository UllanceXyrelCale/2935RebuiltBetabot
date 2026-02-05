package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import static edu.wpi.first.units.Units.RPM;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;

public class FeederSubsystem extends SubsystemBase {
  public enum Speed {
    FEED(5000);

    private final double rpm;

    private Speed (double rpm) {
      this.rpm = rpm;
    }

    public AngularVelocity angularVelocity() {
      return RPM.of(rpm);
    }
  }

  private final TalonFX feederMotor;
  private final VelocityVoltage velocityRequest;
  private final VoltageOut voltageRequest;

  /** Creates a new FloorSubsystem. */
  public FeederSubsystem() {
    feederMotor = new TalonFX(31);

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Set motor outputs
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Current limit
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Feedback
    config.Feedback.SensorToMechanismRatio = 1.0;

    // Set up PID
    config.Slot0.kP = 1;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.Slot0.kV = 12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond);

    // Applies the actual configs to the motor
    feederMotor.getConfigurator().apply(config); 
    velocityRequest = new VelocityVoltage(0).withSlot(0);
    voltageRequest = new VoltageOut(0);
  }

   public void setFeeder(Speed speed) {
      feederMotor.setControl(velocityRequest.withVelocity(speed.angularVelocity()));
  }

  public void stopFeeder() {
    feederMotor.setControl(voltageRequest.withOutput(Volts.of(0)));
  }

  // Simple inline command
  public Command feederCommand() {
    return startEnd(() -> setFeeder(Speed.FEED), () -> stopFeeder());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
