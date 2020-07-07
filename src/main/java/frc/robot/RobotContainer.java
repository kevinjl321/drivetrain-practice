package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.RunCommand;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    public final DriveTrain m_driveTrain = DriveTrain.getInstance(); 
  
    // The driver's controller
    private Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);
  
    // Secondary controllers
    private Joystick m_firstStick = new Joystick(OIConstants.firstStickPort);

    private Joystick m_secondStick = new Joystick(OIConstants.secondStickPort);


    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_driveTrain.setDefaultCommand(new RunCommand(
        () -> m_driveTrain.tankDrive(m_mainStick.getRawAxis(1), m_mainStick.getRawAxis(4), m_mainStick.getRawAxis(2), m_mainStick.getRawAxis(3)),
        m_driveTrain)
      );

      
    }
}
  


      

      

     
  
