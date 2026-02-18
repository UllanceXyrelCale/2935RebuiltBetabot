package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StartShooter extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final LimelightSubsystem limelightSubsystem; // null if hardcoded
  private final double targetRPS; // ignored if using limelight

  // Hardcoded RPS
  public StartShooter(ShooterSubsystem shooterSubsystem, double targetRPS) {
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = null;
    this.targetRPS = targetRPS;

    addRequirements(shooterSubsystem);
  }

  // Limelight-calculated RPS
  public StartShooter(ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.targetRPS = 0; // unused

    addRequirements(shooterSubsystem); // no limelight requirement, just reading it
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (limelightSubsystem != null) {
      shooterSubsystem.setVelocity(limelightSubsystem.getShooterRPS());
    } else {
      shooterSubsystem.setVelocity(targetRPS);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}