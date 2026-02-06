package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import static edu.wpi.first.units.Units.RPM;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.MotorIDConstants;

public class FeederSubsystem extends SubsystemBase {
  public enum Speed {
    FEED(2500);

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
    feederMotor = new TalonFX(MotorIDConstants.feederMotorID);

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
    feederMotor.getConfigurator().apply(config); 
    velocityRequest = new VelocityVoltage(FeederConstants.velocityVoltage).withSlot(FeederConstants.slot);
    voltageRequest = new VoltageOut(FeederConstants.voltageOut);
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
