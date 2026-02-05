package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

public class IntakeSubsystem extends SubsystemBase {

  public enum Speed {
    STOP(0),
    INTAKE(5000);

    private final double rpm;

    Speed(double rpm) {
      this.rpm = rpm;
    }

    public AngularVelocity angularVelocity() {
      return RPM.of(rpm);
    }
  }

  private final TalonFX intakeMotor;
  private final VelocityVoltage velocityRequest;
  private final VoltageOut voltageRequest;

  public IntakeSubsystem() {
    intakeMotor = new TalonFX(40);

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor output
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Current limits
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = 1.0;

    config.Slot0.kP = 1;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond);

    intakeMotor.getConfigurator().apply(config);

    velocityRequest = new VelocityVoltage(0).withSlot(0);
    voltageRequest = new VoltageOut(0);
  }

  public void setIntake(Speed speed) {
    intakeMotor.setControl(
        velocityRequest.withVelocity(speed.angularVelocity())
    );
  }

  public void stopIntake() {
    intakeMotor.setControl(
        voltageRequest.withOutput(Volts.of(0))
    );
  }

  public Command intakeCommand() {
    return startEnd(
        () -> setIntake(Speed.INTAKE),
        () -> stopIntake()
    );
  }
}