package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class IndexerSubsystem extends SubsystemBase {
  private final TalonFX floorMotor;
  private final TalonFX feederMotor;
  
  // Separate velocity requests for each motor
  private final VelocityVoltage floorVelocityRequest;
  private final VelocityVoltage feederVelocityRequest;
  
  public IndexerSubsystem() {
    floorMotor = new TalonFX(32);
    feederMotor = new TalonFX(31);
    
    // Initialize separate requests for each motor
    floorVelocityRequest = new VelocityVoltage(0).withSlot(0);
    feederVelocityRequest = new VelocityVoltage(0).withSlot(0);

    floorMotor.getConfigurator().apply(Configs.floorMotor.floorConfig);
    feederMotor.getConfigurator().apply(Configs.feederMotor.feederConfig);
  }

  /** Set floor motor velocity in RPS */
  public void setFloor(double rps) {
    floorMotor.setControl(floorVelocityRequest.withVelocity(rps));
  }

  /** Set feeder motor velocity in RPS */
  public void setFeeder(double rps) {
    feederMotor.setControl(feederVelocityRequest.withVelocity(rps)); 
  }

  public void setIndexer(double rps) {
    floorMotor.setControl(floorVelocityRequest.withVelocity(rps));
    feederMotor.setControl(floorVelocityRequest.withVelocity(rps));
  }

  /** Stop floor motor */
  public void stopFloor() {
    floorMotor.stopMotor();
  }

  /** Stop feeder motor */
  public void stopFeeder() {
    feederMotor.stopMotor();
  }

  /** Stop both motors */
  public void stopIndexer() {
    stopFloor();
    stopFeeder();
  }

  /** Get floor motor velocity in RPS */
  public double getFloorSpeed() {
    return floorMotor.getVelocity().getValueAsDouble();
  }

  /** Get feeder motor velocity in RPS */
  public double getFeederSpeed() {
    return feederMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Indexer/Floor RPS", getFloorSpeed());
    SmartDashboard.putNumber("Indexer/Feeder RPS", getFeederSpeed());
  }
}