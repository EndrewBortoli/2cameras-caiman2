package frc.robot.commands.launcherjoint;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LauncherConstants;

public class ChangeSetpointLauncherCmd extends Command {

  public double jointLauncherSetpoint;

  public ChangeSetpointLauncherCmd(double kLaunchingPosition) {
    jointLauncherSetpoint = kLaunchingPosition;
  }

  @Override
  public void initialize() {
    // System.out.println("Iniciando troca de setpoint");
  }

  @Override
  public void execute() {
    LauncherConstants.kLauncherJointMotorSetPoint = jointLauncherSetpoint;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Setpoint Alterado");
    System.out.println("Setpoint Launcher: " + LauncherConstants.kLauncherJointMotorSetPoint);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
