// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private final DifferentialDrive m_robotDrive;
  private final Joystick m_leftStick;
  private final Joystick m_rightStick;
  private double startTime; 
  private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    m_rightMotor.setInverted(true);

    m_robotDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);

    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    startTime = Timer.getTimestamp();
  }

  @Override
  public void autonomousPeriodic() {
    
    double time = Timer.getFPGATimestamp();

  if (time - startTime < 3) {
    m_leftMotor.set(0.6);
    m_rightMotor.set(0.6);
  } else {
    m_leftMotor.set(0);
    m_rightMotor.set(0);
  }

  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
   // Read joystick Y-axes (forward = positive)
   double leftRaw  = -m_leftStick.getRawAxis(1);
   double rightRaw = -m_rightStick.getRawAxis(1);

   // Compute speed and turn for arcade-style tank reconstruction
   double speed = (leftRaw + rightRaw) / 2.0 * 0.6;  // 60% max forward/back
   double turn  = (leftRaw - rightRaw) / 2.0 * 0.3;  // 30% max turnp
   
   // Compute left and right motor powers
   double leftPower  = speed + turn;
   double rightPower = speed - turn;

   // Optional: deadband to prevent drift
   if (Math.abs(leftPower) < 0.05) leftPower = 0;
   if (Math.abs(rightPower) < 0.05) rightPower = 0;

   // Set the motors directly
   m_leftMotor.set(leftPower);
   m_rightMotor.set(rightPower);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
