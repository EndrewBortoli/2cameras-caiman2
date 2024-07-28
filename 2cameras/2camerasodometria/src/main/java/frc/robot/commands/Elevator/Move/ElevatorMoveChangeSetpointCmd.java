package frc.robot.commands.Elevator.Move;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorMoveChangeSetpointCmd extends Command {

  public double MoveElevatorSetPoint;

  public ElevatorMoveChangeSetpointCmd(double kElevatorMoveSetpoint) {
    MoveElevatorSetPoint = kElevatorMoveSetpoint;
  }

  @Override
  public void initialize() {
    // System.out.println("Iniciando troca de setpoint");
  }

  @Override
  public void execute() {
    ElevatorConstants.ElevatorMovementSetPoint = MoveElevatorSetPoint;
  }

  @Override
  public void end(boolean interrupted) {
    // System.out.println("Setpoint Alterado");
    // System.out.println("Setpoint Elevator: " + ElevatorConstants.ElevatorMovementSetPoint);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
