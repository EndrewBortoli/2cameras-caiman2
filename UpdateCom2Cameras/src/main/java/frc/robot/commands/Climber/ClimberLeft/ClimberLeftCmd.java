package frc.robot.commands.Climber.ClimberLeft;
   
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.JacLib.JoystickOI;
import frc.robot.Constants.*;
import frc.robot.subsystems.Climber.ClimberLeftSubsystem;

public class ClimberLeftCmd extends Command {

private final ClimberLeftSubsystem climberSubsystem;
private final PIDController pidController;
private final Supplier<Double> setpointFunction;
private static double ClimberSetpoint;
public final Joystick m_driverController = new Joystick(JoystickOI.kDriverControllerPort);

public ClimberLeftCmd(ClimberLeftSubsystem climberSubsystem, Supplier<Double> setpointFunction) {
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
  if(m_driverController.getRawButton(JoystickOI.LB)){
    ClimberConstants.kLeftClimberSetpoint -=2.4;}
  if(m_driverController.getRawButton(JoystickOI.RT)){
    ClimberConstants.kLeftClimberSetpoint  +=1.6;}

    pidController.setSetpoint(ClimberSetpoint);
    
    double leftClimberSpeed = pidController.calculate(climberSubsystem.getLeftClimberPosition());

    if(ClimberConstants.kLeftClimberSetpoint > 0) {
      ClimberConstants.kLeftClimberSetpoint = 0;
    }

    if(ClimberConstants.kLeftClimberSetpoint < -227) {
      ClimberConstants.kLeftClimberSetpoint = -227;
    }
  
    if (leftClimberSpeed > ClimberConstants.kClimberMaxVelocity){
      climberSubsystem.setLeftClimber(ClimberConstants.kClimberMaxVelocity);
    } else if (leftClimberSpeed < ClimberConstants.kClimberMinVelocity){
      climberSubsystem.setLeftClimber(ClimberConstants.kClimberMinVelocity);
    }

  
  climberSubsystem.setLeftClimber(leftClimberSpeed);
  
  //  SmartDashboard.putNumber("Setpoint AngleElevator", pidController.getSetpoint());  
  //  SmartDashboard.putNumber("Right Climber Speed", leftClimberSpeed);
  //  SmartDashboard.putNumber("Left Climber Speed", leftClimberSpeed);
  }
  
  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if(pidController.atSetpoint()){return true;}
    else{return false;}
  }
}
