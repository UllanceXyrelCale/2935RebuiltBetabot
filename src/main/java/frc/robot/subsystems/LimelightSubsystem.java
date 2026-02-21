
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.APTree;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-aptag");
  private static APTree distanceLookup = new APTree();
  private static APTree speedLookup = new APTree();

  // Creates a table for the different velocities from different distances
  private static final double [][] DISTANCE_DATA = {
    {21.10, 0.93}, // {tY, Distance}
    {14.81, 1.133}, 
    {11.56, 1.59},
    {8.65, 1.85},
    {5.84, 2.157},
    {4.29, 2.357},
  };

  private static final double SPEED_DATA[][] = {
    {0.93, 56.7},// {Distance, Velocity} 
    {1.133, 57}, 
    {1.59, 57.5},
    {1.85, 59.1},
    {2.157, 60},
    {2.357, 62.7},
  };

  public LimelightSubsystem() {
    distanceLookup.InsertValues(DISTANCE_DATA);
    speedLookup.InsertValues(SPEED_DATA);
  }

  ///////////////////////////////////////////////////////////
  //            Data Used for Other Subsystems             //
  ///////////////////////////////////////////////////////////
  public double getTagDistance() {
    return distanceLookup.GetValue(getTY());
  }

  public double getShooterRPS() {
    return speedLookup.GetValue(getTagDistance());
  }

  public double getTurnAngle() {

    if (hasValidTarget()) {
      return 0;
    }

    if (Math.abs(getTX()) <= VisionConstants.kAngleTolerance) {
      return 0;
    }

    return getTX();
  }

  ///////////////////////////////////////////////////////////
  //                    Limelight Data                     //
  ///////////////////////////////////////////////////////////
  public double getDoubleEntry(String entry) {
    return limelight.getEntry(entry).getDouble(0);
  }

  public double[] getArraryEntry(String entry) {
    return limelight.getEntry(entry).getDoubleArray(new double[6]);
  }

  public boolean hasValidTarget() {
    return limelight.getEntry("tv").getDouble(0) == 1;
  }

  public double getTID() {
    return getDoubleEntry("tid");
  }

  public double getTA() {
    return getDoubleEntry("ta");
  }

  public double getTX() {
    return getDoubleEntry("tx");
  }

  public double getTY() {
    return getDoubleEntry("ty");
  }

  public boolean isTagAllowedForRotation() {
    int id = (int) Math.round(getTID());
    return id == 2 || id == 5 || id == 10;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("HAS TARGET", hasValidTarget());

    if (hasValidTarget()) {
      SmartDashboard.putNumber("tid", getTID());
      SmartDashboard.putNumber("ta", getTA());
      SmartDashboard.putNumber("ty", getTY());
      SmartDashboard.putNumber("tx", getTX());
    }
  }

  
}
