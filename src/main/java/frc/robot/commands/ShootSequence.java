package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSequence extends SequentialCommandGroup {  // Changed to Sequential!
  public ShootSequence(
      ShooterSubsystem shooter,
      FeederSubsystem feeder,
      FloorSubsystem floor,
      LimelightSubsystem limelight
  ) {
    addCommands(
      // Step 1: Spin up shooter and WAIT until at speed
      new StartShooter(shooter, limelight.getShooterRPS())
        .until(() -> shooter.atTargetSpeed(2.0)),
      
      // Step 2: Once at speed, run everything in parallel
      new ParallelCommandGroup(
        // Keep shooter running
        new StartShooter(shooter, limelight.getShooterRPS()),
        
        // Pulse floor and feeder
        new SequentialCommandGroup(
          new ParallelCommandGroup(
            new StartFloor(floor, 15),
            new StartFeeder(feeder, 20)
          ),

          new ParallelCommandGroup(
            new StartFloor(floor, 0),
            new StartFeeder(feeder, 0)
          )
        ).repeatedly()
      )
    );
  }
}