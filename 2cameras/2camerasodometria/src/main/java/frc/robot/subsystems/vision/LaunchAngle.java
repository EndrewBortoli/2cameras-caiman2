// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaunchAngle extends SubsystemBase {
  public PhotonLL limelight;

  private static final double rightDistanceLauncher1 = 2.82; //Distancia na qual o launcher acertou em centimetros            Perto
  private static final double rightAngleLauncher1 = -25.6; //Angulo na qual o launcher acertou em Graus 

  private static final double rightDistanceLauncher2 = 3.28; //Distancia na qual o launcher acertou em centimetros             Distante
  private static final double rightAngleLauncher2 = -28.4; //Angulo na qual o launcher acertou em Graus

  private static final double rightDistanceElevator1 = 2.82; //Distancia na qual o launcher acertou em centimetros            Perto
  private static final double rightAngleElevator1 = -25.6; //Angulo na qual o launcher acertou em Graus 

  private static final double rightDistanceElevator2 = 3.28; //Distancia na qual o launcher acertou em centimetros             Distante
  private static final double rightAngleElevator2 = -28.4; //Angulo na qual o launcher acertou em Graus

  public LaunchAngle() {
    this.limelight = PhotonLL.getInstance();
  }

  @Override
  public void periodic() {
  }

  public Double AngleLauncher(){
    // Calcula o ângulo com base na distância atual medida
    double currentDistance = limelight.getXDistance();
    if (currentDistance > 300) {
          double angle = (rightAngleLauncher2 * rightDistanceLauncher2) / currentDistance; // Proporcionalmente ajustado com base na distância atual
    return angle;
    }

    else {
          double angle = (rightAngleLauncher1 * rightDistanceLauncher1) / currentDistance; // Proporcionalmente ajustado com base na distância atual
    return angle;
    }    

  }

  public Double AngleElevator(){
    // Calcula o ângulo com base na distância atual medida
    double currentDistance = limelight.getXDistance();
    if (currentDistance > 300) {
          double angle = (rightAngleElevator2 * rightDistanceElevator2) / currentDistance; // Proporcionalmente ajustado com base na distância atual
    return angle;
    }

    else {
          double angle = (rightAngleElevator1 * rightDistanceElevator1) / currentDistance; // Proporcionalmente ajustado com base na distância atual
    return angle;
    }    

  }


}
   