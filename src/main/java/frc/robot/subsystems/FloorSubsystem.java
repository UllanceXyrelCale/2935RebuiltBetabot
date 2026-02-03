package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import static edu.wpi.first.units.Units.RPM;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FloorSubsystem extends SubsystemBase {
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

  private final TalonFX floorMotor;
  private final VelocityVoltage velocityRequest;
  private final VoltageOut voltageRequest;

  /** Creates a new FloorSubsystem. */
  public FloorSubsystem() {
    floorMotor = new TalonFX(31);

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Set motor outputs
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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
    floorMotor.getConfigurator().apply(config); 
    velocityRequest = new VelocityVoltage(0).withSlot(0);
    voltageRequest = new VoltageOut(0);
  }

   public void setFloor(Speed speed) {
      floorMotor.setControl(velocityRequest.withVelocity(speed.angularVelocity()));
  }

  public void stopFloor() {
    floorMotor.setControl(voltageRequest.withOutput(Volts.of(0)));
  }

  // Simple inline command
  public Command floorCommand() {
    return startEnd(() -> setFloor(Speed.FEED), () -> stopFloor());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
