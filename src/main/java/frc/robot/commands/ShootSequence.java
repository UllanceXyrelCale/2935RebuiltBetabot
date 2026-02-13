package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSequence extends SequentialCommandGroup {  // Changed to Sequential!
  public ShootSequence(
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      LimelightSubsystem limelight
  ) {
    addCommands(
      // Step 1: Spin up shooter and WAIT until at speed
      new StartShooter(shooter, limelight)
        .until(() -> shooter.atTargetSpeed(2.0)),
      
      // Step 2: Once at speed, run everything in parallel
      new ParallelCommandGroup(
        // Keep shooter running
        new StartShooter(shooter, limelight),
        
        // Pulse floor and feeder
        new SequentialCommandGroup(
          new StartIndexer(indexer, 20).withTimeout(.25),
          new StartIndexer(indexer, 0).withTimeout(0.10)
        ).repeatedly()
      )
    );
  }
}