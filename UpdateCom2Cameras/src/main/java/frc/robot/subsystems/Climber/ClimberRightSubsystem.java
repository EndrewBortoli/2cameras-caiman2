package frc.robot.subsystems.Climber;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberRightSubsystem extends SubsystemBase {

  public final CANSparkFlex rightClimberMotor = new CANSparkFlex((int) ClimberConstants.kRightClimberId, MotorType.kBrushless);
  private RelativeEncoder rightClimberEncoder = rightClimberMotor.getEncoder();

  public double getRightClimberPosition(){
    return rightClimberEncoder.getPosition();
  }

  public void ClimberSubsystem() {

    rightClimberMotor.setSmartCurrentLimit(80);    

  }

  @Override
  public void periodic() {

    // SmartDashboard.putNumber("ClimberRightPosition", getRightClimberPosition());

  }

  

  public void setRightClimber(double speed) {
    rightClimberMotor.set(speed);
  }
}