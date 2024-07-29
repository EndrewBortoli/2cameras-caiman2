package frc.robot.commands.Elevator.Angle;
   
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.JacLib.JoystickOI;
import frc.robot.Constants.*;
import frc.robot.subsystems.Elevator.ElevatorAngleSubsystem;

public class ElevatorAngleCmd extends Command {

private final ElevatorAngleSubsystem elevatorAngleSubsystem;
private final PIDController pidController;
private final Supplier<Double> setpointFunction;
private static double elevatorAngleSetpoint;
public static boolean kVelocityChange;
public final Joystick m_operatorController = new Joystick(JoystickOI.kOperatorControllerPort);
// public static final ChangeSetpointLauncherCmd CHANGE_SETPOINT_LAUNCHER_CMD = new ChangeSetpointLauncherCmd( kLaunchingPosition);
// public int goal;

// public goal = ChangeSetpointLauncherCmd(double kLaunchingPosition);

public ElevatorAngleCmd(ElevatorAngleSubsystem elevatorAngleSubsystem, Supplier<Double> setpointFunction) {
  this.setpointFunction = setpointFunction;
  this.elevatorAngleSubsystem = elevatorAngleSubsystem;
  this.pidController = new PIDController(
    ElevatorConstants.AnglekP, 
    ElevatorConstants.AnglekI, 
    ElevatorConstants.AnglekD
   );

    addRequirements(elevatorAngleSubsystem);

  }

  @Override
  public void initialize() {
    pidController.reset();
  }

  @Override
  public void execute() {
  elevatorAngleSetpoint = setpointFunction.get();
  if(m_operatorController.getRawButton(OperatorConstants.kElevatorAngleDown)){
    ElevatorConstants.AngulationElevatorSetPoint -=0.01;}
  if(m_operatorController.getRawButton(OperatorConstants.kElevatorAngleUp)){
    ElevatorConstants.AngulationElevatorSetPoint +=0.01;}
  // if(m_operatorController.getRawButton(OperatorConstants.BACK)){
  //   ElevatorConstants.AngulationElevatorSetPoint +=0.01;}

    pidController.setSetpoint(elevatorAngleSetpoint);
    
    double speed = pidController.calculate(elevatorAngleSubsystem.getRightAnglePosition());

    if(ElevatorConstants.AngulationElevatorSetPoint > PositionConstants.elevatorJointPositionUpLimit) {
      ElevatorConstants.AngulationElevatorSetPoint = PositionConstants.elevatorJointPositionUpLimit;
    }
    if (ElevatorConstants.AngulationElevatorSetPoint < PositionConstants.elevatorJointPositionDownLimit) {
      ElevatorConstants.AngulationElevatorSetPoint = PositionConstants.elevatorJointPositionDownLimit;
    }
  
    
      if(speed > ElevatorConstants.kUpAngulationVelocityMax) {
          speed = ElevatorConstants.kUpAngulationVelocityMax;
      }
      if(speed < -ElevatorConstants.kDownAngulationVelocityMax) {
          speed = -ElevatorConstants.kDownAngulationVelocityMax;
      }


  elevatorAngleSubsystem.setMotor(speed);
  
  // SmartDashboard.putNumber("Setpoint AngleElevator", pidController.getSetpoint());  
  // SmartDashboard.putNumber("Velocity AngleElevator", speed);
  // SmartDashboard.putBoolean("AngleElevatorChangeVelocity", kVelocityChange);
  // SmartDashboard.putBoolean("Up Command", m_operatorController.getRawButtonPressed(OperatorConstants.kSubwoofer));
  // SmartDashboard.putBoolean("Down Command", m_operatorController.getRawButtonPressed(OperatorConstants.kFloorIntake));

  elevatorAngleSubsystem.setMotor(speed);

  
  }
  
  @Override
  public void end(boolean interrupted) {
  //  System.out.println("AngleElevator - Chegou na posicao");
  }

  @Override
  public boolean isFinished() {
    if(pidController.atSetpoint()){return true;}
    else{return false;}
  }
}
