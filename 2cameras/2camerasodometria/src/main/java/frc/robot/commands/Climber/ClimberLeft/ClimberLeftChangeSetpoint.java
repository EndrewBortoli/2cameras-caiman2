package frc.robot.commands.Climber.ClimberLeft;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;;

public class ClimberLeftChangeSetpoint extends Command {

  public double ClimberSetpoint;

  public ClimberLeftChangeSetpoint(double kClimberSetpoint) {
    ClimberSetpoint = kClimberSetpoint;
  }

  @Override
   public void initialize() {
    // System.out.println("Iniciando troca de setpoint");
  }

  @Override
  public void execute() {
    ClimberConstants.kRightClimberSetpoint = ClimberSetpoint;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Setpoint Alterado");
    System.out.println("Setpoint CLimber: " + ClimberConstants.kRightClimberSetpoint);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
