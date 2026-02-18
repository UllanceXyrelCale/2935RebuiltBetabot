package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.APPID;
import frc.robot.utils.Calculations;
import frc.robot.utils.Pose;

public class TurnToAngle extends Command {
  private final DriveSubsystem driveSubsystem;
  private final LimelightSubsystem limelightSubsystem; // null if hardcoded
  private final APPID turnPID;

  private double targetAngleDeg;           // not final anymore — limelight updates it each loop
  private final double angleToleranceDeg;

  private static final double kTurnP = 0.02;
  private static final double kTurnI = 0.0;
  private static final double kTurnD = 0.0;
  private static final double kMaxRot = 0.25;

  // Hardcoded angle
  public TurnToAngle(DriveSubsystem driveSubsystem, double targetAngleDeg, double angleToleranceDeg) {
    this.driveSubsystem = driveSubsystem;
    this.limelightSubsystem = null;
    this.targetAngleDeg = targetAngleDeg;
    this.angleToleranceDeg = angleToleranceDeg;

    this.turnPID = new APPID(kTurnP, kTurnI, kTurnD, angleToleranceDeg);
    this.turnPID.setMaxOutput(kMaxRot);

    addRequirements(driveSubsystem);
  }

  public TurnToAngle(DriveSubsystem driveSubsystem, double targetAngleDeg) {
    this(driveSubsystem, targetAngleDeg, 2.0);
  }

  // Limelight mode — dynamically tracks tx each loop
  public TurnToAngle(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem, double angleToleranceDeg) {
    this.driveSubsystem = driveSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.targetAngleDeg = 0; // updated dynamically in execute()
    this.angleToleranceDeg = angleToleranceDeg;

    this.turnPID = new APPID(kTurnP, kTurnI, kTurnD, angleToleranceDeg);
    this.turnPID.setMaxOutput(kMaxRot);

    addRequirements(driveSubsystem); // limelight not required, just reading it
  }

  public TurnToAngle(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem) {
    this(driveSubsystem, limelightSubsystem, 2.0);
  }

  @Override
  public void initialize() {
    turnPID.reset();
  }

  @Override
  public void execute() {
    // Update target angle from limelight if in limelight mode
    if (limelightSubsystem != null) {
      if (!limelightSubsystem.hasValidTarget()) {
        driveSubsystem.drive(0, 0, 0, true);
        return;
      }
      targetAngleDeg = driveSubsystem.getHeading() - limelightSubsystem.getTX();
    }

    Pose currentPose = driveSubsystem.getPose();
    double currentAngle = Calculations.normalizeAngle360(currentPose.getAngle());
    double targetAngle  = Calculations.normalizeAngle360(targetAngleDeg);
    double angleError   = Calculations.shortestAngularDistance(targetAngle, currentAngle);

    turnPID.setDesiredValue(0);
    double rotCmd = turnPID.calculate(-angleError);
    driveSubsystem.drive(0.0, 0.0, rotCmd, true);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0.0, 0.0, 0.0, true);
  }

  @Override
  public boolean isFinished() {
    // End immediately if limelight mode and target is lost
    if (limelightSubsystem != null && !limelightSubsystem.hasValidTarget()) return true;

    Pose currentPose = driveSubsystem.getPose();
    double currentAngle = Calculations.normalizeAngle360(currentPose.getAngle());
    double targetAngle  = Calculations.normalizeAngle360(targetAngleDeg);
    double angleError   = Math.abs(Calculations.shortestAngularDistance(targetAngle, currentAngle));
    return angleError <= angleToleranceDeg;
  }
}