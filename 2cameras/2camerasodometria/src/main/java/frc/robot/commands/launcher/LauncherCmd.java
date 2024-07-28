// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher.LauncherSubsystem;

public class LauncherCmd extends Command {
  private final LauncherSubsystem launcherSubsystem;
  private String state;

  public LauncherCmd(LauncherSubsystem launcherSubsystem, String state) {
    this.launcherSubsystem = launcherSubsystem;
    this.state = state;
}

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
   launcherSubsystem.setLauncher(state);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
