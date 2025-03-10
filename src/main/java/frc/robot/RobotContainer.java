// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.Constants.DriverControllerConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorControllerConstants;
import frc.robot.commands.CoralIntakeCmd;
import frc.robot.commands.MoveArmToSetpoint;
import frc.robot.commands.MoveElevatorToSetpoint;
import frc.robot.commands.limelightCommands.alignXandYLeftCamera;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  // Call subsystems
  private final DriveSubsystem s_driveSubsystem = new DriveSubsystem();
  private final CoralIntakeSubsystem s_CoralIntakeSubsystem = new CoralIntakeSubsystem();
  private final ArmSubsystem s_ArmSubsystem = new ArmSubsystem();
  private final ElevatorSubsystem s_ElevatorSubsystem = new ElevatorSubsystem();
  private final VisionSubsystem s_VisionSubsystem = new VisionSubsystem();

  // Setup Driver Controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(DriverControllerConstants.kDriverControllerPort);

  // Setup Operator Controller
  private final CommandXboxController m_operatorController = 
      new CommandXboxController(OperatorControllerConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    s_driveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> s_driveSubsystem.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), DriverControllerConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), DriverControllerConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), DriverControllerConstants.kDriveDeadband),
                true),
            s_driveSubsystem));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
   
    //________________________________________DRIVER BUTTONS____________________________________________________________//

    // INTAKE
    m_driverController.leftBumper().whileTrue(
      new CoralIntakeCmd(s_CoralIntakeSubsystem, CoralIntakeConstants.kCoralIntakeSpeed)
    );

    m_driverController.leftBumper().whileFalse(
      new CoralIntakeCmd(s_CoralIntakeSubsystem, CoralIntakeConstants.kCoralNoSpeed)
    );

    // OUTAKE
    m_driverController.rightBumper().whileTrue(
    new CoralIntakeCmd(s_CoralIntakeSubsystem, CoralIntakeConstants.kCoralOutakeSpeed)
    );

    m_driverController.rightBumper().whileFalse(
      new CoralIntakeCmd(s_CoralIntakeSubsystem, CoralIntakeConstants.kCoralNoSpeed)
    );

    //_______________________________________OPERATOR BUTTONS___________________________________________________________//


    // GO TO HOME POSITION




    // GO TO FEEDER POSITION




    // GO TO TRAVEL POSITION




    // GO TO LEVEL 1




    // GO TO LEVEL 2




    // GO TO LEVEL 3




    // GO TO LEVEL 4


    m_operatorController.x().onTrue(

      new SequentialCommandGroup(
        new MoveArmToSetpoint(s_ArmSubsystem, ArmConstants.kLevel4),
        new MoveElevatorToSetpoint(s_ElevatorSubsystem, ElevatorConstants.kLevel4)
      )

    );

    // Feeder Position

    m_operatorController.a().onTrue(

      new SequentialCommandGroup(
        new MoveElevatorToSetpoint(s_ElevatorSubsystem, ElevatorConstants.kFeederStation),
        new MoveArmToSetpoint(s_ArmSubsystem, ArmConstants.kFeederStation)
      )

    );

    m_operatorController.y().onTrue(

      new SequentialCommandGroup(
        new MoveArmToSetpoint(s_ArmSubsystem, ArmConstants.kLevel3),
        new MoveElevatorToSetpoint(s_ElevatorSubsystem, ElevatorConstants.kLevel3)
      )

    );

    m_operatorController.b().onTrue(

      new SequentialCommandGroup(
        new MoveArmToSetpoint(s_ArmSubsystem, ArmConstants.kLevel2),
        new MoveElevatorToSetpoint(s_ElevatorSubsystem, ElevatorConstants.kHome)
      )

    );

    m_operatorController.leftBumper().onTrue(
      new SequentialCommandGroup(
        new MoveArmToSetpoint(s_ArmSubsystem, ArmConstants.kTravel),
        new MoveElevatorToSetpoint(s_ElevatorSubsystem, ElevatorConstants.kTravel)
      )
    );

    m_operatorController.rightBumper().onTrue(
      new SequentialCommandGroup(
        new MoveElevatorToSetpoint(s_ElevatorSubsystem, ElevatorConstants.kHome),
        new MoveArmToSetpoint(s_ArmSubsystem, ArmConstants.kHome)
        )
    );


    // autonomous alignment commands 

    m_driverController.b().onTrue(
      new alignXandYLeftCamera(s_driveSubsystem, s_VisionSubsystem, 0, false, 21, -11, 0, 0)
    ); 

    m_driverController.b().onFalse(
      new alignXandYLeftCamera(s_driveSubsystem, s_VisionSubsystem, 0, true, 21, -11, 0, 0)
    ); 

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

}
