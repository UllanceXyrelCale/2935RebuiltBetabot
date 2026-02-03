package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SubsystemCommands {
  private final FeederSubsystem feederSubsystem;
  private final FloorSubsystem floorSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public SubsystemCommands(
    FeederSubsystem feederSubsystem,
    FloorSubsystem floorSubsystem,
    ShooterSubsystem shooterSubsystem
  ) {
    this.feederSubsystem = feederSubsystem;
    this.floorSubsystem = floorSubsystem;
    this.shooterSubsystem = shooterSubsystem;
  }

  public Command shootManually() {
    return shooterSubsystem.shooterCommand(5000)
    .andThen(feed())
    .handleInterrupt(()-> shooterSubsystem.setShooter(0));
  }

  public Command feed() {
    return Commands.sequence(
      Commands.waitSeconds(0.25),
      Commands.parallel(
        feederSubsystem.feederCommand(),
        Commands.waitSeconds(0.125)
        .andThen(floorSubsystem.floorCommand()) // did not add along with line because no intake
      )
    );
  }
}