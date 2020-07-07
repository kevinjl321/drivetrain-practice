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
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.PortConstantsFinal;

public class DriveTrain{

  private static DriveTrain m_instance;

  private final CANSparkMax m_frontLeft, m_middleLeft, m_backLeft, m_frontRight, m_middleRight, m_backRight;
  private CANEncoder m_leftEncoder, m_rightEncoder;
  private CANPIDController m_leftController, m_rightController;

  private DifferentialDrive m_drive;

  public PIDController m_turnController;

  private double straightAngle; 
  private boolean straightAngleSet;

  private Timer m_timer;

  private double grandKP;

  public final DifferentialDriveKinematics kDriveKinematics;
  // private Compressor m_comp;

  public static DriveTrain getInstance(){
    if (m_instance == null){
      m_instance = new DriveTrain();
    }
    return m_instance;
  }

  public DriveTrain() {

    grandKP = DriveConstants.kP_DRIVE;

    if (Constants.isFinal){
      m_frontLeft = new CANSparkMax(PortConstantsFinal.FRONT_LEFT_DRIVE, MotorType.kBrushless);
      m_middleLeft = new CANSparkMax(PortConstantsFinal.MIDDLE_LEFT_DRIVE, MotorType.kBrushless);
      m_backLeft = new CANSparkMax(PortConstantsFinal.BACK_LEFT_DRIVE, MotorType.kBrushless); 
      
      m_frontRight = new CANSparkMax(PortConstantsFinal.FRONT_RIGHT_DRIVE, MotorType.kBrushless);
      m_middleRight = new CANSparkMax(PortConstantsFinal.MIDDLE_RIGHT_DRIVE, MotorType.kBrushless);
      m_backRight = new CANSparkMax(PortConstantsFinal.BACK_RIGHT_DRIVE, MotorType.kBrushless);  

    } else {
      m_frontLeft = new CANSparkMax(PortConstants.FRONT_LEFT_DRIVE, MotorType.kBrushless);
      m_middleLeft = new CANSparkMax(PortConstants.MIDDLE_LEFT_DRIVE, MotorType.kBrushless);
      m_backLeft = new CANSparkMax(PortConstants.BACK_LEFT_DRIVE, MotorType.kBrushless); 
      
      m_frontRight = new CANSparkMax(PortConstants.FRONT_RIGHT_DRIVE, MotorType.kBrushless);
      m_middleRight = new CANSparkMax(PortConstants.MIDDLE_RIGHT_DRIVE, MotorType.kBrushless);
      m_backRight = new CANSparkMax(PortConstants.BACK_RIGHT_DRIVE, MotorType.kBrushless);  
    }
    
    m_frontLeft.restoreFactoryDefaults();
    m_middleLeft.restoreFactoryDefaults();
    m_backLeft.restoreFactoryDefaults();

    m_frontRight.restoreFactoryDefaults();
    m_middleRight.restoreFactoryDefaults();
    m_backRight.restoreFactoryDefaults();

    m_frontLeft.follow(m_backLeft);
    m_middleLeft.follow(m_backLeft);
    
    m_frontRight.follow(m_backRight);
    m_middleRight.follow(m_backRight);

    m_leftEncoder = m_backLeft.getEncoder();
    m_rightEncoder = m_backRight.getEncoder();

    // m_leftEncoder.setPositionConversionFactor(factor);

    // important
    // m_leftEncoder.setVelocityConversionFactor(factor);

    m_leftController = m_backLeft.getPIDController();
    m_rightController = m_backRight.getPIDController();

    m_drive = new DifferentialDrive(m_backLeft, m_backRight);
 
    m_turnController = new PIDController(AutoConstants.TURN_KP, AutoConstants.TURN_KI, AutoConstants.TURN_KD);
    m_turnController.enableContinuousInput(AutoConstants.MIN_INPUT, AutoConstants.MAX_INPUT);
    m_turnController.setIntegratorRange(AutoConstants.MIN_INGL, AutoConstants.MAX_INGL);
    m_turnController.setTolerance(AutoConstants.TOLERANCE);
    m_turnController.setSetpoint(0);

    SmartDashboard.putData("Turn Controller", m_turnController);

    straightAngle = 0.0;
    straightAngleSet = false;

    m_timer = new Timer();

    kDriveKinematics = new DifferentialDriveKinematics(AutoConstants.kTRACKWIDTH);


    // SmartDashboard.putNumber("TURN_KP", AutoConstants.TURN_KP);
    //SmartDashboard.putNumber("kS", AutoConstants.kS);
    SmartDashboard.putNumber("KP", grandKP);

  }

  /**
   * Manual drive of the robot.
   *
   * @return The pose.
   */

  public void voltageDrive(double voltage) // moves each gearbox accordingly
  {
    double sign = Math.signum(voltage);
    m_backLeft.setVoltage(sign*AutoConstants.kS_CONCRETE + voltage);
    m_backRight.setVoltage(sign*AutoConstants.kS_CONCRETE + voltage);
    // SmartDashboard.putNumber("voltage in turn", voltage);
  }
  
  /**
   * Manual Drive of the robot.
   *
   */
  public void tankDrive(double x, double z, double lowLeft, double lowRight){

    double adaptedZ = 0;

    
    straightAngleSet = false;
    if(lowLeft > 0 || lowRight > 0){
      adaptedZ = DriveConstants.TURN_FACTOR*DriveConstants.LOW_DPI*(lowRight-lowLeft);
    } else {
      adaptedZ = DriveConstants.TURN_FACTOR*z;
    }

    SmartDashboard.putNumber("Z", adaptedZ);

    SmartDashboard.putNumber("straightAngle", straightAngle);
  
    double[] outputs = scale(x, adaptedZ);
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
  public double[] scale(double x, double z){    
    double rawLeftInput = -x+z;
    double rawRightInput = x+z;

    // forward: left pos, right neg
    double leftDirection = Math.signum(rawLeftInput);
    double rightDirection = Math.signum(rawRightInput);

    double leftOutput = AutoConstants.kS*leftDirection + AutoConstants.kV_CONCRETE *(rawLeftInput);
    double rightOutput = AutoConstants.kS*rightDirection + AutoConstants.kV_CONCRETE *(rawRightInput);

    double[] rawInputs = {rawLeftInput, rawRightInput};
    double[] directions = {leftDirection, rightDirection};
    double[] outputs = {leftOutput, rightOutput};

    SmartDashboard.putNumberArray("inputs", rawInputs);
    SmartDashboard.putNumberArray("directions", directions);
    SmartDashboard.putNumberArray("outputs", outputs);

    // when you run the SPARK MAX in voltage mode there is no control loop, it is still running open loop
    // https://www.chiefdelphi.com/t/frc-characterization-output-driven-results/374592/9
    // m_leftController.setReference(leftOutput, ControlType.kVoltage);
    // m_rightController.setReference(rightOutput, ControlType.kVoltage);
    
    double finalOutputLeft = cubic(leftOutput);
    double finalOutputRight = cubic(rightOutput);
    
    double[] finalOutputs = {finalOutputLeft, finalOutputRight};

    SmartDashboard.putNumberArray("finaloutputs", finalOutputs);

    return finalOutputs;
  }

  /**
   * Reduces speeds at lower inputs.
   *
   * @return 0.5x^3 + 0.5x.
   */
  public double cubic(double input){
    return 0.5*Math.pow(input, 3) + 0.5*Math.pow(input, 1);
  }

    /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }
}