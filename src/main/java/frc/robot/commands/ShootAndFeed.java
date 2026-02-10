package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAndFeed extends ParallelCommandGroup {
  public ShootAndFeed(
      ShooterSubsystem shooter,
      double shooterRPS,
      FloorSubsystem floor,
      FeederSubsystem feeder
  ) {
    addCommands(
      // Branch 1: shooter runs immediately and keeps running
      new SetShooterVelocity(shooter, shooterRPS),

      // Branch 2: wait, then run floor + feeder together
      new SequentialCommandGroup(
        new WaitCommand(1),
        new ParallelCommandGroup(
          new StartFloor(floor),
          new StartFeeder(feeder)
        )
      )
    );
  }
}