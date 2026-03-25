
// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

// import frc.robot.subsystems.roller.RollerSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PrepareShotCommand;
import frc.robot.commands.SubsystemCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // Subsystems
        private final Drive drive;
        private Vision vision;

        private final Intake intake = new Intake();
        private final Floor floor = new Floor();
        private final Feeder feeder = new Feeder();
        private final Shooter shooter = new Shooter();
        private final Hood hood = new Hood();
        private final Hanger hanger = new Hanger();

        // Controller
        private final CommandXboxController controller;

        // Dashboard inputs
        private LoggedDashboardChooser<Command> autoChooser;

        private final SubsystemCommands subsystemCommands;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {


                controller = new CommandXboxController(0);
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations

                                drive = new Drive(new GyroIOPigeon2(),
                                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                                new ModuleIOTalonFX(TunerConstants.BackRight));
                                // Vision
                                vision = new Vision(drive::addVisionMeasurement,
                                                new VisionIOLimelight("limelight-bop", drive::getRotation));

                                break;
                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations
                                drive = new Drive(new GyroIO() {
                                }, new ModuleIOSim(TunerConstants.FrontLeft),
                                                new ModuleIOSim(TunerConstants.FrontRight),
                                                new ModuleIOSim(TunerConstants.BackLeft),
                                                new ModuleIOSim(TunerConstants.BackRight));

                                vision = new Vision(drive::addVisionMeasurement,
                                                new VisionIOLimelight("limelight-bop", drive::getRotation));
                                break;

                        default:
                                // Replayed robot, disable IO implementations
                                drive = new Drive(new GyroIO() {
                                }, new ModuleIO() {
                                }, new ModuleIO() {
                                }, new ModuleIO() {
                                },
                                                new ModuleIO() {
                                                });

                                vision = new Vision(drive::addVisionMeasurement,
                                                new VisionIOLimelight("limelight-bop", drive::getRotation));
                                break;
                }

                subsystemCommands = new SubsystemCommands(
                drive,
                intake,
                floor,
                feeder,
                shooter,
                hood,
                hanger
                );

                //register named commands
                NamedCommands.registerCommand("aimAndShoot", subsystemCommands.aimAndShoot().withTimeout(10));
                NamedCommands.registerCommand("deployIntake", intake.intakeCommand());
                NamedCommands.registerCommand("stowIntake", intake.runOnce(() -> intake.set(Intake.Position.STOWED)));
                // Set up auto routines
                autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

                // Configure the button bindings
                configureButtonBindings();

        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses
         * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
         * passing it to a
         * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {


 

                // Default command, normal field-relative drive
                drive.setDefaultCommand(
                                DriveCommands.joystickDrive(
                                                drive,
                                                () -> -controller.getLeftY(),
                                                () -> -controller.getLeftX(),
                                                () -> -controller.getRightX()));

                RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
                                .onTrue(intake.homingCommand())
                                .onTrue(hanger.homingCommand());

                controller.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
                controller.rightBumper().whileTrue(subsystemCommands.shootManually());
                controller.leftTrigger().whileTrue(intake.intakeCommand());
                controller.leftBumper().onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));

                controller.povUp().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
                controller.povDown().onTrue(hanger.positionCommand(Hanger.Position.HUNG));
        }

        
        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.get();
        }

        public Pose2d getPose2D() {
                return drive.getPose();
        }

        public void teleopPeriodic() {
        }
}
