package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAndFeed extends SequentialCommandGroup {  // Changed to Sequential!
  public ShootAndFeed(
      ShooterSubsystem shooter,
      FloorSubsystem floor,
      FeederSubsystem feeder
  ) {
    addCommands(
      // Step 1: Spin up shooter and WAIT until at speed
      new SetShooterVelocity(shooter)
        .until(() -> shooter.atTargetSpeed(10)),
      
      // Step 2: Once at speed, run everything in parallel
      new ParallelCommandGroup(
        // Keep shooter running
        new SetShooterVelocity(shooter),
        
        // Pulse floor and feeder
        new SequentialCommandGroup(
          new ParallelCommandGroup(
            new StartFloor(floor, 50),
            new StartFeeder(feeder, 50)
          ).withTimeout(0.25),
          new ParallelCommandGroup(
            new StartFloor(floor, 0),
            new StartFeeder(feeder, 0)
          ).withTimeout(0.10)
        ).repeatedly()
      )
    );
  }
}