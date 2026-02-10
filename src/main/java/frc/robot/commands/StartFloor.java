package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorSubsystem;

public class StartFloor extends Command {
  private final FloorSubsystem floorSubsystem;
  private final double floorRPS;
   
  public StartFloor(FloorSubsystem floorSubsystem, double floorRPS) {
    this.floorSubsystem = floorSubsystem;
    this.floorRPS = floorRPS;
    addRequirements(floorSubsystem);
    }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    floorSubsystem.setFloor(floorRPS);
  }

  @Override
  public void end(boolean interrupted) {
    floorSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}