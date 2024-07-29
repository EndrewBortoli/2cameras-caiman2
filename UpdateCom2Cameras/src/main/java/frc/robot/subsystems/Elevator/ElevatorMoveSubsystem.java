// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class ElevatorMoveSubsystem extends SubsystemBase {
  private final CANSparkMax elevatorMovementMotor = new CANSparkMax((int) ElevatorConstants.kElevatorMovementMotorId, CANSparkLowLevel.MotorType.kBrushless);
  private RelativeEncoder moveEncoder = elevatorMovementMotor.getEncoder();

  public double getMovePosition(){
    return moveEncoder.getPosition();
  }

  public double getCurrent(){
    return elevatorMovementMotor.getOutputCurrent();
  }

  public ElevatorMoveSubsystem() {
    elevatorMovementMotor.setSmartCurrentLimit(35);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Move Position", getMovePosition());
    // SmartDashboard.putNumber("ActualCurrent", elevatorMovementMotor.getAppliedOutput());
  }

  public void setMotor(double speed){
    elevatorMovementMotor.set(speed);
  }

  public boolean atPosition(double pos){
    if(Math.abs(getMovePosition()-pos)<1){
      return true;
    }else{return false;}
  }

}
