package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SubsystemCommands {
  private final FeederSubsystem feederSubsystem;
  private final FloorSubsystem floorSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  public SubsystemCommands(
    FeederSubsystem feederSubsystem,
    FloorSubsystem floorSubsystem,
    ShooterSubsystem shooterSubsystem,
    IntakeSubsystem intakeSubsystem
  ) {
    this.feederSubsystem = feederSubsystem;
    this.floorSubsystem = floorSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
  }

  // ------ Public Commands --------- //
  public Command shootManually() {
    return shooterSubsystem.shooterCommand(2500)
    .andThen(feed())
    .handleInterrupt(()-> shooterSubsystem.setShooter(0));
  }

  public Command intakeManually() {
    return Commands.sequence(
      intakeSubsystem.intakeCommand(),
      floorSubsystem.floorCommand());
  }

  // ------ Helper Commands --------- //
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