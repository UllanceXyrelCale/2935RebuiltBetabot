package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class StartIntake extends Command {
  public final IntakeSubsystem intakeSubsystem;
  public final double intakeTargetRPS;

  public StartFeeder(IntakeSubsystem intakeSubsystem, double intakeTargetRPS) {
    this.intakeSubsystem = intakeSubsystem;
    this.intakeTargetRPS = intakeTargetRPS;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    feederSubsystem.setVelocity(feederTargetRPS);
  }

  @Override
  public void end(boolean interrupted) {
    feederSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
