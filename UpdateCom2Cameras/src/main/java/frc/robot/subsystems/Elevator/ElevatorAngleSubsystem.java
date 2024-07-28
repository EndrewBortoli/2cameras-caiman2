// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LauncherConstants;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class ElevatorAngleSubsystem extends SubsystemBase {
  private final CANSparkMax rightAngleMotor = new CANSparkMax((int) ElevatorConstants.kElevatorRightAngulation, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax leftAngleMotor = new CANSparkMax((int) ElevatorConstants.kElevatorLeftAngulation, CANSparkLowLevel.MotorType.kBrushless);  
  private RelativeEncoder rightEncoder = rightAngleMotor.getEncoder();
  private RelativeEncoder leftEncoder = leftAngleMotor.getEncoder();

  public double getRightAnglePosition(){
    return rightEncoder.getPosition();
  }

  public double getLeftAnglePosition(){
    return leftEncoder.getPosition();
  }


  public ElevatorAngleSubsystem() {
    leftAngleMotor.follow(rightAngleMotor, true);
  }

  @Override
  public void periodic() {
    
    if (getRightAnglePosition() < -1.6 & getRightAnglePosition() > -1.7 ){
        ElevatorConstants.kUpAngulationVelocityMax = 0.15;
    }
    else {
      ElevatorConstants.kUpAngulationVelocityMax = 0.35;
    }

    if (getRightAnglePosition() < 0 & getRightAnglePosition() > -0.2){
        ElevatorConstants.kDownAngulationVelocityMax = 0.15;
    }
    else {
      ElevatorConstants.kDownAngulationVelocityMax = 0.3;
    }
    // SmartDashboard.putNumber("RightAngle Elevator Position", getRightAnglePosition());
    // SmartDashboard.putNumber("LeftAngle Elevator Position", getLeftAnglePosition());    
    // SmartDashboard.putNumber("ActualCurrent", rightAngleMotor.getAppliedOutput());
  }

  public void setMotor(double speed){
    rightAngleMotor.set(speed);
  }

  public boolean atPosition(double pos){
    if(Math.abs(getRightAnglePosition()-pos)<1){
      return true;
    }else{return false;}
  }

}
