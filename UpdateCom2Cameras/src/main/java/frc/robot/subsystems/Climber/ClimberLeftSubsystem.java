package frc.robot.subsystems.Climber;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberLeftSubsystem extends SubsystemBase {

  private final CANSparkFlex leftClimberMotor = new CANSparkFlex((int) ClimberConstants.kLeftClimberId, MotorType.kBrushless);
  private RelativeEncoder leftClimberEncoder = leftClimberMotor.getEncoder();

  public double getLeftClimberPosition(){
    return leftClimberEncoder.getPosition();
  }



  public void ClimberSubsystem() {

    leftClimberMotor.setSmartCurrentLimit(80);
    

  }

  @Override
  public void periodic() {

    // SmartDashboard.putNumber("ClimberLeftPosition", getLeftClimberPosition());

  }

  public void setLeftClimber(double speed) {
    leftClimberMotor.set(speed);
  }

}