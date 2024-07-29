package frc.robot.commands.Elevator.Move;
   
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.JacLib.JoystickOI;
import frc.robot.Constants.*;
import frc.robot.subsystems.Elevator.ElevatorMoveSubsystem;

public class ElevatorMoveCmd extends Command {

private final ElevatorMoveSubsystem elevatorMoveSubsystem;
private final PIDController pidController;
private final Supplier<Double> setpointFunction;
private static double elevatorMoveSetpoint;
public final Joystick m_operatorController = new Joystick(JoystickOI.kOperatorControllerPort);
public final Joystick m_DriverController = new Joystick(JoystickOI.kDriverControllerPort);

public ElevatorMoveCmd(ElevatorMoveSubsystem elevatorMoveSubsystem, Supplier<Double> setpointFunction) {
  this.setpointFunction = setpointFunction;
  this.elevatorMoveSubsystem = elevatorMoveSubsystem;
  this.pidController = new PIDController(
    ElevatorConstants.ElevatorMomeventkP, 
    ElevatorConstants.ElevatorMomeventkI, 
    ElevatorConstants.ElevatorMomeventkD
   );

    addRequirements(elevatorMoveSubsystem);

  }

  @Override
  public void initialize() {
    pidController.reset();
  }

  @Override
  public void execute() {
  elevatorMoveSetpoint = setpointFunction.get();
  if(m_DriverController.getRawButton(OperatorConstants.kElevatorMoveUp)){
    ElevatorConstants.ElevatorMovementSetPoint -=1;}
  if(m_DriverController.getRawButton(OperatorConstants.kElevatorMoveDown)){
    ElevatorConstants.ElevatorMovementSetPoint +=1;}

    pidController.setSetpoint(elevatorMoveSetpoint);
    
    double speed = pidController.calculate(elevatorMoveSubsystem.getMovePosition());

    if(ElevatorConstants.ElevatorMovementSetPoint < 0) {
      ElevatorConstants.ElevatorMovementSetPoint = 0;
    }
     else if (ElevatorConstants.ElevatorMovementSetPoint > 165) {
       ElevatorConstants.ElevatorMovementSetPoint = 165;
    }

    if (ElevatorConstants.kMoveVelocityMax > 0.5 ) {
      ElevatorConstants.kMoveVelocityMax = 0.5;
    }
    
     else if (-ElevatorConstants.kMoveVelocityMax < -0.5) {
      ElevatorConstants.kMoveVelocityMax = -0.5;
     }
  
  elevatorMoveSubsystem.setMotor(speed);
  
  //  SmartDashboard.putNumber("Setpoint MoveElevator", pidController.getSetpoint());  
  //  SmartDashboard.putNumber("Velocity MoveElevator", speed);
  //  SmartDashboard.putNumber("Current", elevatorMoveSubsystem.getCurrent());

  }
  
  @Override
  public void end(boolean interrupted) {
  //  System.out.println("MoveElevator - Chegou na posicao");
  }
  
  @Override
  public boolean isFinished() {
    if(pidController.atSetpoint()){return true;}
    else{return false;}
  }
}
