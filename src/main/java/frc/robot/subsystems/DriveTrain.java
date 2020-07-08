/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.PortConstantsFinal;

public class DriveTrain extends SubsystemBase {

  private static DriveTrain m_instance;

  private final CANSparkMax m_frontLeft, m_backLeft, m_frontRight, m_backRight;
  private CANEncoder m_leftEncoder, m_rightEncoder;

  private DifferentialDrive m_drive;

  public final DifferentialDriveKinematics kDriveKinematics;
  // private Compressor m_comp;

  public static DriveTrain getInstance() {
    if (m_instance == null) {
      m_instance = new DriveTrain();
    }
    return m_instance;
  }

  public DriveTrain() {

    if (Constants.isFinal) {
      m_frontLeft = new CANSparkMax(PortConstantsFinal.FRONT_LEFT_DRIVE, MotorType.kBrushless);
      m_backLeft = new CANSparkMax(PortConstantsFinal.BACK_LEFT_DRIVE, MotorType.kBrushless);

      m_frontRight = new CANSparkMax(PortConstantsFinal.FRONT_RIGHT_DRIVE, MotorType.kBrushless);
      m_backRight = new CANSparkMax(PortConstantsFinal.BACK_RIGHT_DRIVE, MotorType.kBrushless);

    } else {
      m_frontLeft = new CANSparkMax(PortConstants.FRONT_LEFT_DRIVE, MotorType.kBrushless);
      m_backLeft = new CANSparkMax(PortConstants.BACK_LEFT_DRIVE, MotorType.kBrushless);

      m_frontRight = new CANSparkMax(PortConstants.FRONT_RIGHT_DRIVE, MotorType.kBrushless);
      m_backRight = new CANSparkMax(PortConstants.BACK_RIGHT_DRIVE, MotorType.kBrushless);
    }

    m_frontLeft.restoreFactoryDefaults();
    m_backLeft.restoreFactoryDefaults();

    m_frontRight.restoreFactoryDefaults();
    m_backRight.restoreFactoryDefaults();

    m_frontLeft.follow(m_backLeft);

    m_frontRight.follow(m_backRight);

    m_leftEncoder = m_backLeft.getEncoder();
    m_rightEncoder = m_backRight.getEncoder();

    // m_leftEncoder.setPositionConversionFactor(factor);

    // important
    // m_leftEncoder.setVelocityConversionFactor(factor);

    m_drive = new DifferentialDrive(m_backLeft, m_backRight);

    kDriveKinematics = new DifferentialDriveKinematics(AutoConstants.kTRACKWIDTH);

  }

  /**
   * Manual drive of the robot.
   *
   * @return The pose.
   */

  public void voltageDrive(double voltage) // moves each gearbox accordingly
  {
    double sign = Math.signum(voltage);
    m_backLeft.setVoltage(sign * AutoConstants.kS_CONCRETE + voltage);
    m_backRight.setVoltage(sign * AutoConstants.kS_CONCRETE + voltage);
    // SmartDashboard.putNumber("voltage in turn", voltage);
  }

  /**
   * Manual Drive of the robot.
   *
   */
  public void tankDrive(double x, double z, double lowLeft, double lowRight) {

    double[] outputs = scale(x, z);
    m_backLeft.setVoltage(outputs[0]);
    m_backRight.setVoltage(outputs[1]);

    // important: might stop jittering!s
    m_drive.feed();
  }

  /**
   * Takes input and accounts for kS and kV
   *
   * @return the corrected output.
   */
  public double[] scale(double x, double z) {
    double rawLeftInput = -x + z;
    double rawRightInput = x + z;

    // forward: left pos, right neg
    double leftDirection = Math.signum(rawLeftInput);
    double rightDirection = Math.signum(rawRightInput);

    double leftOutput = AutoConstants.kS * leftDirection + AutoConstants.kV_CONCRETE * (rawLeftInput);
    double rightOutput = AutoConstants.kS * rightDirection + AutoConstants.kV_CONCRETE * (rawRightInput);

    double[] rawInputs = { rawLeftInput, rawRightInput };
    double[] directions = { leftDirection, rightDirection };
    double[] outputs = { leftOutput, rightOutput };

    SmartDashboard.putNumberArray("inputs", rawInputs);
    SmartDashboard.putNumberArray("directions", directions);
    SmartDashboard.putNumberArray("outputs", outputs);

    // when you run the SPARK MAX in voltage mode there is no control loop, it is
    // still running open loop
    // https://www.chiefdelphi.com/t/frc-characterization-output-driven-results/374592/9
    // m_leftController.setReference(leftOutput, ControlType.kVoltage);
    // m_rightController.setReference(rightOutput, ControlType.kVoltage);

    double finalOutputLeft = cubic(leftOutput);
    double finalOutputRight = cubic(rightOutput);

    double[] finalOutputs = { finalOutputLeft, finalOutputRight };

    SmartDashboard.putNumberArray("finaloutputs", finalOutputs);

    return finalOutputs;
  }

  /**
   * Reduces speeds at lower inputs.
   *
   * @return 0.5x^3 + 0.5x.
   */
  public double cubic(double input) {
    return 0.5 * Math.pow(input, 3) + 0.5 * Math.pow(input, 1);
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  public void setDefaultCommand(RunCommand runCommand) {
  }
}