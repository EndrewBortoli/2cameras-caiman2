package frc.robot.commands.launcherjoint;
   
import frc.robot.subsystems.JointLauncher.JointLauncherSubsystem;
import frc.robot.subsystems.vision.LaunchAngle;
import frc.robot.subsystems.vision.PhotonLL;

import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.commands.Elevator.Angle.ElevatorAngleChangeSetpointCmd;


public class JointLauncherCommand extends Command {

private final JointLauncherSubsystem jointLauncherSubsystem;
private final PIDController pidController;
private final Supplier<Double> setpointFunction;
private static double jointLauncherSetpoint;
private LaunchAngle angle = new LaunchAngle();

public final Joystick m_operatorController = new Joystick(OIConstants.kOperatorControllerPort);

  public JointLauncherCommand(JointLauncherSubsystem jointLauncherSubsystem, Supplier<Double> setpointFunction) {
    this.setpointFunction = setpointFunction;
    this.jointLauncherSubsystem = jointLauncherSubsystem;
    this.pidController = new PIDController(
    LauncherConstants.kPIDAngulationMotorKp, 
    LauncherConstants.kPIDAngulationMotorKi, 
    LauncherConstants.kPIDAngulationMotorKd
    );

    addRequirements(jointLauncherSubsystem);

  }

  @Override
  public void initialize() {
    pidController.reset();
  }

  @Override
  public void execute() {
  jointLauncherSetpoint = setpointFunction.get();

  if(m_operatorController.getRawButton(OIConstants.LB)){
    LauncherConstants.kLauncherJointMotorSetPoint -=0.1;
  }

  if(m_operatorController.getRawButton(OIConstants.RB)){
    LauncherConstants.kLauncherJointMotorSetPoint +=0.1;
  }

  if(m_operatorController.getRawButton(OIConstants.BACK)){
    new ElevatorAngleChangeSetpointCmd(PositionConstants.elevatorJointPositionPodium);
    LauncherConstants.kLauncherJointMotorSetPoint = angle.AngleLauncher();
  }
  

    pidController.setSetpoint(jointLauncherSetpoint) ;

    // SmartDashboard.putNumber("Setpoint JointLauncher", pidController.getSetpoint());
    // SmartDashboard.putNumber("limelight.getXDistance()", angle.XDIstance());    

    double speed = pidController.calculate(jointLauncherSubsystem.getJointLauncherLeftMotorPosition());

    if(speed>LauncherConstants.kLauncherDownJointMotorMaxSpeed){
      speed=LauncherConstants.kLauncherDownJointMotorMaxSpeed;
    }
    if(speed<-LauncherConstants.kLauncherUpJointMotorMaxSpeed){
      speed=-LauncherConstants.kLauncherUpJointMotorMaxSpeed;
    }


    // if(LauncherConstants.kLauncherJointMotorSetPoint > -1.5) {
    //   LauncherConstants.kLauncherJointMotorSetPoint = -1.5;
    // }    else if(LauncherConstants.kLauncherJointMotorSetPoint < -40){
    //     LauncherConstants.kLauncherJointMotorSetPoint = -40;
    // }
   
  jointLauncherSubsystem.setMotor(speed);

  // SmartDashboard.putNumber("Launcher Joint Speed", speed);
  }
  
  @Override
  public void end(boolean interrupted) {
  //  System.out.println("JointLauncher - Chegou na posicao");
  }

  @Override
  public boolean isFinished() {
    if(pidController.atSetpoint()){return true;}
    else{return false;}
  }
}
