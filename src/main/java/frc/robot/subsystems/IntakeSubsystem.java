package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.MotorIDConstants;

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
    intakeMotor = new TalonFX(MotorIDConstants.intakeMotorID);

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Set motor outputs
    config.MotorOutput.NeutralMode = FeederConstants.neutralMode;
    config.MotorOutput.Inverted = FeederConstants.isInverted;

    // Current limit
    config.CurrentLimits.SupplyCurrentLimit = FeederConstants.supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = FeederConstants.supplyCurrentLimitEnable;

    // Feedback
    config.Feedback.SensorToMechanismRatio = FeederConstants.sensorToMechanismRatio;

    // Set up PID
    config.Slot0.kP = FeederConstants.kP;
    config.Slot0.kI = FeederConstants.kI;
    config.Slot0.kD = FeederConstants.kD;
    config.Slot0.kV = FeederConstants.kV;

    // Applies the actual configs to the motor
    intakeMotor.getConfigurator().apply(config); 
    velocityRequest = new VelocityVoltage(FeederConstants.velocityVoltage).withSlot(FeederConstants.slot);
    voltageRequest = new VoltageOut(FeederConstants.voltageOut);
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