package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.APTree;
import frc.robot.utils.Pose;

public class LimelightSubsystem extends SubsystemBase {

  private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-aptag");
  private static APTree tagToDistanceLookup = new APTree();
  private static APTree distanceToSpeedLookup = new APTree();
  private static double distanceMeters = 0;
  private static final double CAMERA_TO_CENTRE = 0.349;

  public double getDistanceMeters() {
    return distanceMeters;
  }

  // -------------------------------------------------------
  //   Per-tag TY → Distance tables
  // -------------------------------------------------------
  private static final double[][] T10_DISTANCE_DATA = {
    {21.10, 0.93},
    {14.81, 1.133},
    {11.56, 1.59},
    {8.65,  1.85},
    {5.84,  2.157},
    {4.29,  2.357},
  };

  private static final double[][] T2_DISTANCE_DATA = {
    {21.10, 10.34},
    {14.81, 10.567},
    {11.56, 10.867},
    {8.65,  11.5},
    {5.84,  12.34},
    {4.29,  12.8},
  };

  private static final double[][] T5_DISTANCE_DATA = {
    {21.10, 10.93},
    {14.81, 11.133},
    {11.56, 11.59},
    {8.65,  11.85},
    {5.84,  12.157},
    {4.29,  12.357},
  };

  private static final double[][] T13_DISTANCE_DATA = {
    {21.10, 0.93},
    {14.81, 1.133},
    {11.56, 1.59},
    {8.65,  1.85},
    {5.84,  2.157},
    {4.29,  2.357},
  };

  private static final double[][] T1_DISTANCE_DATA = {
    {21.10, 0.93},
    {14.81, 1.133},
    {11.56, 1.59},
    {8.65,  1.85},
    {5.84,  2.157},
    {4.29,  2.357},
  };

  // Distance → Shooter RPS
  private static final double[][] SPEED_DATA = {
    {0.93,  56.7},
    {1.133, 57.0},
    {1.59,  57.5},
    {1.85,  59.1},
    {2.157, 60.0},
    {2.357, 62.7},
  };

  public LimelightSubsystem() {
    distanceToSpeedLookup.InsertValues(SPEED_DATA);
  }

  // -------------------------------------------------------
  //   Tag Classification
  // -------------------------------------------------------

  /** Returns the TY→Distance table for a given tag ID, or null if unsupported. */
  public double[][] getDataForTag(double IDNum) {
    switch ((int) Math.round(IDNum)) {
      case 10: return T10_DISTANCE_DATA;
      case 2:  return T2_DISTANCE_DATA;
      case 5:  return T5_DISTANCE_DATA;
      case 13: return T13_DISTANCE_DATA;
      case 1:  return T1_DISTANCE_DATA;
      default: return null;
    }
  }

  /** Tags that are valid targets for auto-aiming/rotation. */
  public boolean isAimTag(double IDNum) {
    int id = (int) Math.round(IDNum);
    return id == 2 || id == 5 || id == 10 || id == 13;
  }

  /** Tags that are valid for resetting odometry pose. */
  public boolean isPoseResetTag(double IDNum) {
    int id = (int) Math.round(IDNum);
    return id == 1;
  }

  // -------------------------------------------------------
  //   Computed Values
  // -------------------------------------------------------

  /** Returns distance to the given tag ID using the current TY reading. */
  public double getTagDistance(double IDNum) {
    double[][] data = getDataForTag(IDNum);
    if (data == null) return 0;

    tagToDistanceLookup = new APTree();
    tagToDistanceLookup.InsertValues(data);
    return tagToDistanceLookup.GetValue(getTY());
  }

  /** Returns the shooter RPS for the given tag ID based on distance. */
  public double getShooterRPS(double IDNum) {
    if (getDataForTag(IDNum) == null) return 30.0;
    return distanceToSpeedLookup.GetValue(getTagDistance(IDNum));
  }

  /** Returns TX if the robot needs to turn, 0 if already aligned or no target. */
  public double getTurnAngle() {
    if (!hasValidTarget()) return 0;
    if (Math.abs(getTX()) <= VisionConstants.kAngleTolerance) return 0;
    return getTX();
  }

  // -------------------------------------------------------
  //   Raw Limelight Network Table Accessors
  // -------------------------------------------------------

  public double getDoubleEntry(String entry) {
    return limelight.getEntry(entry).getDouble(0);
  }

  public double[] getArraryEntry(String entry) {
    return limelight.getEntry(entry).getDoubleArray(new double[6]);
  }

  public boolean hasValidTarget() {
    return limelight.getEntry("tv").getDouble(0) == 1;
  }

  public double getTID() { return getDoubleEntry("tid"); }
  public double getTA()  { return getDoubleEntry("ta"); }
  public double getTX()  { return getDoubleEntry("tx"); }
  public double getTY()  { return getDoubleEntry("ty"); }

  public Pose getPoseFromTag(double robotYawDeg) {

    if (!hasValidTarget()) {
        return null; // or return current pose instead
    }

    double distance = distanceMeters + CAMERA_TO_CENTRE;  // already updating in periodic
    double tx = getTX();

    // Bearing from field frame
    double bearingRad = Math.toRadians(robotYawDeg - tx);

    // Tag assumed at (0,0)
    double robotX = -distance * Math.cos(bearingRad);
    double robotY = -distance * Math.sin(bearingRad);

    return new Pose(robotX, robotY, robotYawDeg);
}

  // -------------------------------------------------------
  //   Periodic
  // -------------------------------------------------------

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("HAS TARGET", hasValidTarget());

    if (hasValidTarget()) {
      double tid = getTID();
      distanceMeters = getTagDistance(tid);
      SmartDashboard.putNumber("tid",              tid);
      SmartDashboard.putNumber("ta",                getTA());
      SmartDashboard.putNumber("ty",               getTY());
      SmartDashboard.putNumber("tx",               getTX());
      SmartDashboard.putNumber("Distance from tag", getTagDistance(tid));
      SmartDashboard.putBoolean("Is Aim Tag",       isAimTag(tid));
      SmartDashboard.putBoolean("Is Pose Reset Tag", isPoseResetTag(tid));
    }
  }
}