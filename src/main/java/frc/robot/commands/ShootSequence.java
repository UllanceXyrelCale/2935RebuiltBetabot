package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSequence extends SequentialCommandGroup {
  public ShootSequence(
      ShooterSubsystem shooter,
      FeederSubsystem feeder,
      FloorSubsystem floor,
      DriveSubsystem drive,
      LimelightSubsystem limelight
  ) {
    addCommands(

    new ParallelCommandGroup(
            // Step 1: Set the drivetrain to an X to lock into place
      new RunCommand(()-> drive.setX(), drive),

      new SequentialCommandGroup(
        // Step 2: Spin up shooter and WAIT until at speed
        new StartShooter(shooter, 71)
        .until(() -> shooter.atTargetSpeed(2.0)),
        
        
        // Step 3: Once at speed, run everything in parallel
        new ParallelCommandGroup(
          // Keep shooter running
          new StartShooter(shooter, 71),

          // Run floor and feeder
            new ParallelCommandGroup(
              new StartFloor(floor, 25),
              new StartFeeder(feeder, 50)
            )
          )
        )
      )
    );


  }
}