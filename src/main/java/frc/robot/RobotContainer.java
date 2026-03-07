package frc.robot;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SetFeederRPS;
import frc.robot.commands.SetFloorRPS;
import frc.robot.commands.SetIntakeRPS;
import frc.robot.commands.SetPivotPosition;
import frc.robot.commands.SetShooterRPS;
import frc.robot.commands.ShootSequence;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem s_shooterSubsystem = new ShooterSubsystem();
  private final FloorSubsystem s_floorSubsystem = new FloorSubsystem();
  private final FeederSubsystem s_feederSubsystem = new FeederSubsystem();
  private final IntakeSubsystem s_intakeSubsystem = new IntakeSubsystem();
  private final PivotSubsystem s_pivotSubsystem = new PivotSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
      .whileTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value) 
      .whileTrue(new SetShooterRPS(s_shooterSubsystem, 60));

    new JoystickButton(m_driverController, XboxController.Button.kB.value) 
      .whileTrue(new SetFeederRPS(s_feederSubsystem, 50));

    new JoystickButton(m_driverController, XboxController.Button.kX.value) 
      .whileTrue(new SetFloorRPS(s_floorSubsystem, 50));

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
      .whileTrue(new SetPivotPosition(s_pivotSubsystem, 10));

    new JoystickButton(m_driverController, XboxController.Button.kA.value)
      .whileTrue(new SetPivotPosition(s_pivotSubsystem, 110));

    new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.2)
      .whileTrue(new ShootSequence(s_shooterSubsystem, s_feederSubsystem, s_floorSubsystem, m_robotDrive));

    new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.2)
      .whileTrue(new SetIntakeRPS(s_intakeSubsystem, 50));
  
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;

  }

public DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
  }
}