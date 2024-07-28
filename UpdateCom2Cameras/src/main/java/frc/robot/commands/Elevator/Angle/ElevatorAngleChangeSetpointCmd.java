package frc.robot.commands.Elevator.Angle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorAngleChangeSetpointCmd extends Command {

  public double AngulationElevatorSetPoint;

  public ElevatorAngleChangeSetpointCmd(double kElevatorSetpoint) {
    AngulationElevatorSetPoint = kElevatorSetpoint;
  }

  @Override
   public void initialize() {
    // System.out.println("Iniciando troca de setpoint");
  }

  @Override
  public void execute() {   
    ElevatorConstants.AngulationElevatorSetPoint = AngulationElevatorSetPoint;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Setpoint Alterado");
    System.out.println("Setpoint Elevator: " + ElevatorConstants.AngulationElevatorSetPoint);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
