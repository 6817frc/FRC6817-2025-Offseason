// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.Ports;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

	public static final double JOYSTICK_AXIS_THRESHOLD = 0.15;

  public final SwerveDrivetrain drivetrain = new SwerveDrivetrain();

  public final CoralIntake intake = new CoralIntake();

  public final Field2d field = new Field2d();

  public final SendableChooser<Boolean> usePoseEstimateChooser = new SendableChooser<>();

  public boolean useCopilot = false;
  private double speedMultMain = 1;
  private double speedMultSecondary = 1;

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
    drivetrain.setDefaultCommand(new RunCommand(() -> {
      getDriveValues();
      drivetrain.drive(-leftStickX, leftStickY, -rightStickX);
    }, drivetrain));

    
    driverController.a().onTrue(Commands.runOnce(() -> {useCopilot = !useCopilot;})); // button:A - Transfer drive to copilot

    driverController.x().onTrue(Commands.runOnce(() -> drivetrain.setL2Pose())); // button:X - Set the pose based on the tag
    driverController.x().whileTrue(Commands.run(() -> drivetrain.goToIdealPose())); // Go to the pose
    driverController.x().onFalse(Commands.run(() -> drivetrain.resetOffsets())); // Reset
    
    driverController.y().onTrue(Commands.runOnce(() -> drivetrain.zeroHeading())); // button:Y - Reset field orientation
    
    driverController.povDown().onTrue(Commands.runOnce(() -> intake.armL1())); // dPad:Down - L1
    
    driverController.povLeft().onTrue(Commands.runOnce(() -> intake.armL2())); // dPad:Left - L2
    
    driverController.povRight().onTrue(Commands.runOnce(() -> intake.armTravel())); // dPad:Right - Travel
    
    driverController.povUp().onTrue(Commands.runOnce(() -> intake.armIntake())); // dPad:Up - Human Player

    driverController.rightBumper().onTrue(Commands.runOnce(() -> intake.setWheelPower(-0.1))); // rightBumper - Wheel Out
    driverController.rightBumper().onFalse(Commands.runOnce(() -> intake.wheelStop())); // Stop wheel

    driverController.leftBumper().onTrue(Commands.runOnce(() -> intake.setWheelPower(0.1))); // leftBumper - Wheel In
    driverController.leftBumper().onFalse(Commands.runOnce(() -> intake.wheelStop())); // Stop wheel

    driverController.start().whileTrue(Commands.run(() -> drivetrain.faceTowardTag())); // start - face the robot toward the tag
    driverController.start().onFalse(Commands.runOnce(() -> drivetrain.resetOffsets())); // Reset turn offset
  }

  private void getDriveValues() {
    if (useCopilot) {
      speedMultSecondary = driverController.b().getAsBoolean() ? 0.4 : 0.1; // Increase speed if the b button is pressed in copilot mode
      leftStickX = copilotController.getLeftX();
      leftStickY = copilotController.getLeftY();
      rightStickX = copilotController.getRightX();
    } else {
      speedMultSecondary = 1;
      leftStickX = driverController.getLeftX();
      leftStickY = driverController.getLeftY();
      rightStickX = driverController.getRightX();
    }

    speedMultMain = 1 - driverController.getRightTriggerAxis() * 0.75 - driverController.getLeftTriggerAxis() * 0.2;

    leftStickX = MathUtil.applyDeadband(leftStickX, JOYSTICK_AXIS_THRESHOLD) * speedMultSecondary * speedMultMain;
    leftStickY = MathUtil.applyDeadband(leftStickY, JOYSTICK_AXIS_THRESHOLD) * speedMultSecondary * speedMultMain;
    rightStickX = MathUtil.applyDeadband(rightStickX, JOYSTICK_AXIS_THRESHOLD) * speedMultSecondary * speedMultMain;
  }

  public SwerveDrivetrain getDriveTrain() {
    return drivetrain;
  }

  public CoralIntake getIntake() {
    return intake;
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
