// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_robotDrive;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  private final CANSparkMax m_leftFrontMotor = new CANSparkMax(1, MotorType.kBrushed);
  private final CANSparkMax m_rightFrontMotor = new CANSparkMax(2, MotorType.kBrushed);
  private final CANSparkMax m_rearLeftMotor = new CANSparkMax(3, MotorType.kBrushed);
  private final CANSparkMax m_rearRightMotor = new CANSparkMax(4, MotorType.kBrushed);

  // Speed limiter variable
  private static final double SPEED_LIMIT = 0.6; // Adjust this value to set the speed limit

  @Override
  public void robotInit() {
    // Link the rear motors to follow the front motors
    m_rearLeftMotor.follow(m_leftFrontMotor);
    m_rearRightMotor.follow(m_rightFrontMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightFrontMotor.setInverted(true);

    m_robotDrive = new DifferentialDrive(m_leftFrontMotor, m_rightFrontMotor);
    SendableRegistry.addChild(m_robotDrive, m_leftFrontMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightFrontMotor);

    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
  }

  @Override
  public void teleopPeriodic() {
    double leftSpeed = m_leftStick.getY() * SPEED_LIMIT;
    double rightSpeed = m_rightStick.getY() * SPEED_LIMIT;

    m_robotDrive.tankDrive(leftSpeed, rightSpeed);
  }
}
