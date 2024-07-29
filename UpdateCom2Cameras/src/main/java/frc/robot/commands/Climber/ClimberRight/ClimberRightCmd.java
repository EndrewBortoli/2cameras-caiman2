package frc.robot.commands.Climber.ClimberRight;
   
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.JacLib.JoystickOI;
import frc.robot.Constants.*;
import frc.robot.subsystems.Climber.ClimberRightSubsystem;

public class ClimberRightCmd extends Command {

private final ClimberRightSubsystem climberSubsystem;
private final PIDController pidController;
private final Supplier<Double> setpointFunction;
private static double ClimberSetpoint;
public final Joystick m_driverController = new Joystick(JoystickOI.kDriverControllerPort);

public ClimberRightCmd(ClimberRightSubsystem climberSubsystem, Supplier<Double> setpointFunction) {
  this.setpointFunction = setpointFunction;
  this.climberSubsystem = climberSubsystem;
  this.pidController = new PIDController(
    0.2, 
    ClimberConstants.kPIDClimberKi, 
    ClimberConstants.kPIDClimberKd
   );

    addRequirements(climberSubsystem);

  }

  @Override
  public void initialize() {
    pidController.reset();
  }

  @Override
  public void execute() {
    ClimberSetpoint = setpointFunction.get();
  if(m_driverController.getRawButton(JoystickOI.RB)){
    ClimberConstants.kRightClimberSetpoint -=2.4;}
  if(m_driverController.getRawButton(JoystickOI.LT)){
    ClimberConstants.kRightClimberSetpoint  +=1.8;}

    pidController.setSetpoint(ClimberSetpoint);
    
    double rightClimberSpeed = pidController.calculate(climberSubsystem.getRightClimberPosition());

    if(ClimberConstants.kRightClimberSetpoint > 0) {
      ClimberConstants.kRightClimberSetpoint = 0;
    }

    if(ClimberConstants.kRightClimberSetpoint < -245) {
      ClimberConstants.kRightClimberSetpoint = -245;
    }

    if (rightClimberSpeed > ClimberConstants.kClimberMaxVelocity){
      climberSubsystem.setRightClimber(ClimberConstants.kClimberMaxVelocity);
    } else if (rightClimberSpeed < ClimberConstants.kClimberMinVelocity){
      climberSubsystem.setRightClimber(ClimberConstants.kClimberMinVelocity);
    }

  
  climberSubsystem.setRightClimber(rightClimberSpeed);
  
  //  SmartDashboard.putNumber("Setpoint AngleElevator", pidController.getSetpoint());  
  //  SmartDashboard.putNumber("Right Climber Speed", rightClimberSpeed);
  //  SmartDashboard.putNumber("Left Climber Speed", rightClimberSpeed);
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
