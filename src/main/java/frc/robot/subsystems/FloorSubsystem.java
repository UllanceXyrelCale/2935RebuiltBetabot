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
import frc.robot.Constants.FloorConstants;
import frc.robot.Constants.MotorIDConstants;

public class FloorSubsystem extends SubsystemBase {
  public enum Speed {
    FEED(1500);

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
    floorMotor = new TalonFX(MotorIDConstants.floorMotorID);

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Set motor outputs
    config.MotorOutput.NeutralMode = FloorConstants.neutralMode;
    config.MotorOutput.Inverted = FloorConstants.isInverted;

    // Current limit
    config.CurrentLimits.SupplyCurrentLimit = FloorConstants.supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = FloorConstants.supplyCurrentLimitEnable;

    // Feedback
    config.Feedback.SensorToMechanismRatio = FloorConstants.sensorToMechanismRatio;

    // Set up PID
    config.Slot0.kP = FloorConstants.kP;
    config.Slot0.kI = FloorConstants.kI;
    config.Slot0.kD = FloorConstants.kD;
    config.Slot0.kV = FloorConstants.kV;

    // Applies the actual configs to the motor
    floorMotor.getConfigurator().apply(config); 
    velocityRequest = new VelocityVoltage(FloorConstants.velocityVoltage).withSlot(FloorConstants.slot);
    voltageRequest = new VoltageOut(FloorConstants.voltageOut);
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