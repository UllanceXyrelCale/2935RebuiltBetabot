// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class StartIndexer extends Command {
  public final IndexerSubsystem indexerSubsystem;
  public final double indexerTargetRPS;

  public StartIndexer(IndexerSubsystem indexerSubsystem, double indexerTargetRPS) {
    this.indexerSubsystem = indexerSubsystem;
    this.indexerTargetRPS = indexerTargetRPS;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    indexerSubsystem.setIndexer(indexerTargetRPS);
  }

  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stopIndexer();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
