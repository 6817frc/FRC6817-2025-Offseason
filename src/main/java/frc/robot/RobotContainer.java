// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.Ports;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SwerveDrivetrain drivetrain = new SwerveDrivetrain();

  private boolean useCopilot = false;
  private double speedMult = 1;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(Ports.USB.DRIVER_GAMEPAD);
  private final CommandXboxController copilotController = 
      new CommandXboxController(Ports.USB.COPILOT_GAMEPAD);

  private double leftStickX = 0;
  private double leftStickY = 0;
  private double rightStickX = 0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
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
    drivetrain.setDefaultCommand(new RunCommand(()-> {
      getDriveValues();
      drivetrain.drive(leftStickX, leftStickY, rightStickX);
    }));

    driverController.a().onTrue(Commands.runOnce(() -> {useCopilot = !useCopilot;}));
  }

  private void getDriveValues() {
    if (useCopilot) {
      speedMult = 0.2;
      leftStickX = copilotController.getLeftX();
      leftStickY = copilotController.getLeftY();
      rightStickX = copilotController.getRightX();
    } else {
      speedMult = 1;
      leftStickX = driverController.getLeftX();
      leftStickY = driverController.getLeftY();
      rightStickX = driverController.getRightX();
    }

    leftStickX *= speedMult;
    leftStickY *= speedMult;
    rightStickX *= speedMult;
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
