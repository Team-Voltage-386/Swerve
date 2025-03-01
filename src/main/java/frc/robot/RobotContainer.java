// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.Optional;
import java.util.Vector;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
//import frc.robot.TyRap24Constants.*;
import frc.robot.SparkJrConstants.Controller;
import frc.robot.SparkJrConstants.ID;
import frc.robot.SparkJrConstants.DriveTrainConstants;
import frc.robot.Commands.Drive;
import frc.robot.Commands.DriveDistance;
import frc.robot.Commands.DriveFixedVelocity;
import frc.robot.Commands.DriveLeftOrRight;
import frc.robot.Commands.DriveOffset;
import frc.robot.Commands.DriveRange;
import frc.robot.Commands.ResetOdoCommand;
import frc.robot.Commands.StopDrive;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.RangeSensor;
import frc.sim.SimDrivetrain;
import frc.sim.SimLimelight;
import frc.sim.SimTarget;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final Pigeon2 m_gyro = new Pigeon2(ID.kGyro);
    private final Drivetrain m_swerve;
    private final Limelight m_Limelight;
    private final RangeSensor m_range;
    private final SendableChooser<String> autoChooser;
    //private final Elevator m_elevator;

    private ShuffleboardTab m_competitionTab = Shuffleboard.getTab("Competition Tab");
    private GenericEntry m_xVelEntry = m_competitionTab.add("Chassis X Vel", 0).getEntry();
    private GenericEntry m_yVelEntry = m_competitionTab.add("Chassis Y Vel", 0).getEntry();
    private GenericEntry m_gyroAngle = m_competitionTab.add("Gyro Angle", 0).getEntry();
    private GenericEntry m_currentRange = m_competitionTab.add("Range", 0).getEntry();
    private GenericEntry m_commandedXVel = m_competitionTab.add("CommandedVX", 0).getEntry();
    private GenericEntry m_commandedYVel = m_competitionTab.add("CommandedVY", 0).getEntry();
    protected GenericEntry m_driveP = m_competitionTab.add("Drive P Val", DriveTrainConstants.drivePID[0]).getEntry();
    protected GenericEntry m_driveFFStatic = m_competitionTab.add("Drive FF Static", DriveTrainConstants.driveFeedForward[0]).getEntry();
    protected GenericEntry m_driveFFVel = m_competitionTab.add("Drive FF Vel", DriveTrainConstants.driveFeedForward[1]).getEntry();
    protected GenericEntry m_driveAccel = m_competitionTab.add("Drive FF Accel", 0.0).getEntry();
    private GenericEntry m_fixedSpeed = m_competitionTab.add("Fixed Speed", 0).getEntry();
    private StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    private SwerveModuleSB[] mSwerveModuleTelem;

    Command m_driveCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        this.m_gyro.getConfigurator().apply(new MountPoseConfigs().withMountPoseYaw(-90));
        if (RobotBase.isReal()) {
            this.m_swerve = new Drivetrain(m_gyro);
        } else {
            this.m_swerve = new SimDrivetrain();
            Pose3d startPose = new Pose3d(
                1.0, 1.0, 0.0, new Rotation3d(0.0, 0.0, Math.toRadians(0.0)));
            ((SimDrivetrain)m_swerve).setSimPose(startPose);
        }

        SwerveModuleSB[] swerveModuleTelem = {
                new SwerveModuleSB("FR", m_swerve.getFrontRightSwerveModule(), m_competitionTab),
                new SwerveModuleSB("FL", m_swerve.getFrontLeftSwerveModule(), m_competitionTab),
                new SwerveModuleSB("BR", m_swerve.getBackRightSwerveModule(), m_competitionTab),
                new SwerveModuleSB("BL", m_swerve.getBackLeftSwerveModule(), m_competitionTab) };
        mSwerveModuleTelem = swerveModuleTelem;

        if (RobotBase.isReal()) {
            this.m_Limelight = new Limelight();
        } else {
            Vector<SimTarget> targets = new Vector<SimTarget>();
            // Tag 18 coordinates
            SimTarget target = new SimTarget((float) Units.Meters.convertFrom(144, Units.Inches),
                (float) Units.Meters.convertFrom(158.5, Units.Inches), 0.0f);
            targets.add(target);    
            this.m_Limelight = new SimLimelight((SimDrivetrain)this.m_swerve, targets, true);
        }
        this.m_Limelight.setLimelightPipeline(2);

        this.m_range = new RangeSensor(0);

        // Xbox controllers return negative values when we push forward.
        this.m_driveCommand = new Drive(m_swerve);
        this.m_swerve.setDefaultCommand(this.m_driveCommand);

        autoChooser = new SendableChooser<>(); // Default auto will be `Commands.none()'

        configurePathPlanner();
        autoChooser.setDefaultOption("DO NOTHING!", "NO AUTO");
        m_competitionTab.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(7, 0);
        m_competitionTab.add("Drivetrain", this.m_swerve);

        NamedCommands.registerCommand("StopDrive", new StopDrive(m_swerve));

        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        Controller.kDriveController.y().onTrue((new ResetOdoCommand(m_swerve)));
        Controller.kDriveController.rightBumper()
                .onTrue(this.m_swerve.setFieldRelativeCommand(false))
                .onFalse(this.m_swerve.setFieldRelativeCommand(true));

        Controller.kDriveController.leftBumper().onTrue(m_swerve.setDriveMultCommand(0.5))
                .onFalse(m_swerve.setDriveMultCommand(1));
        Controller.kDriveController.a().onTrue(new DriveOffset(m_swerve, m_Limelight, false));
        Controller.kDriveController.b().onTrue(new DriveDistance(m_swerve));
        Controller.kDriveController.x().onTrue(new DriveDistance(m_swerve,
                () -> m_Limelight.getzDistanceMeters() - 0.1, 0));
        Controller.kDriveController.leftBumper().onTrue(new DriveRange(m_swerve, () -> 0.5, () -> m_range.getRange(), 90, 0.2));
        Controller.kDriveController.povUp().whileTrue(new DriveFixedVelocity(m_swerve, 0, () -> m_fixedSpeed.getDouble(0.5)));
        Controller.kDriveController.povDown().whileTrue(new DriveFixedVelocity(m_swerve, 180, () -> m_fixedSpeed.getDouble(0.5)));
        Controller.kDriveController.povLeft().onTrue(new DriveLeftOrRight(m_swerve, m_Limelight, true));
        Controller.kDriveController.povRight().onTrue(new DriveLeftOrRight(m_swerve, m_Limelight, false));
    }

    public Drivetrain getDrivetrain() {
        return this.m_swerve;
    }

    private void configurePathPlanner() {
        autoChooser.addOption("Vision Test", "Vision Test");
        autoChooser.addOption("Drive Straight", "Drive Straight");
        autoChooser.addOption("SwerveTestAuto25", "SwerveTestAuto25");
        autoChooser.addOption("StraightForward", "StraightForward");
        autoChooser.addOption("LimelightTest", "LimelightTest");
    }

    public void startAutonomous() {
        SequentialCommandGroup start = new SequentialCommandGroup(getAutonomousCommand(),
                new DriveOffset(m_swerve, m_Limelight, false));
        start.schedule();
    }

    public Command getAutonomousCommand() {
        if (autoChooser.getSelected().equals("NO AUTO")) {
            return Commands.none();
        }
        System.out.println("getAutoCommand building auto for " + autoChooser.getSelected());
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(autoChooser.getSelected());
            Optional<Pose2d> pose = path.getStartingHolonomicPose();
            if (pose.isPresent()) {
                m_swerve.resetStartingPose(pose.get());
                System.out.println(pose.get());
            } else {
                System.out.println("Error getting PathPlanner pose");
            }
            return AutoBuilder.followPath(path);
        } catch (FileVersionException | IOException | ParseException e) {
            // TODO Auto-generated catch block
            System.err.println("Error loading PathPlanner path");
            e.printStackTrace();
        }
        return new StopDrive(m_swerve);
    }

    public void setTeleDefaultCommand() {
        if (this.m_swerve.getDefaultCommand() == null) {
            this.m_swerve.setDefaultCommand(this.m_driveCommand);
        }
    }

    public void setAutoDefaultCommand() {
        if (this.m_swerve.getDefaultCommand() == null) {
            this.m_swerve.setDefaultCommand(this.m_driveCommand);
        }
    }

    public void clearDefaultCommand() {
        this.m_swerve.removeDefaultCommand();
    }

    public void setPIDConstants() {
        // Configure the drive train tuning constants from the dashboard
        double driveP = m_driveP.getDouble(0.0);
        double driveFFStatic = m_driveFFStatic.getDouble(0.0);
        double driveFFVel = m_driveFFVel.getDouble(0.0);
        double driveFFAccel = m_driveAccel.getDouble(0.0);
        m_swerve.getFrontLeftSwerveModule().getDrivePidController().setP(driveP);
        m_swerve.getFrontLeftSwerveModule().getDriveFeedForward().setKs(driveFFStatic);
        m_swerve.getFrontLeftSwerveModule().getDriveFeedForward().setKv(driveFFVel);
        m_swerve.getFrontLeftSwerveModule().getDriveFeedForward().setKa(driveFFAccel);
        m_swerve.getFrontRightSwerveModule().getDrivePidController().setP(driveP);
        m_swerve.getFrontRightSwerveModule().getDriveFeedForward().setKs(driveFFStatic);
        m_swerve.getFrontRightSwerveModule().getDriveFeedForward().setKv(driveFFVel);
        m_swerve.getFrontRightSwerveModule().getDriveFeedForward().setKa(driveFFAccel);
        m_swerve.getBackLeftSwerveModule().getDrivePidController().setP(driveP);
        m_swerve.getBackLeftSwerveModule().getDriveFeedForward().setKs(driveFFStatic);
        m_swerve.getBackLeftSwerveModule().getDriveFeedForward().setKv(driveFFVel);
        m_swerve.getBackLeftSwerveModule().getDriveFeedForward().setKa(driveFFAccel);
        m_swerve.getBackRightSwerveModule().getDrivePidController().setP(driveP);
        m_swerve.getBackRightSwerveModule().getDriveFeedForward().setKs(driveFFStatic);
        m_swerve.getBackRightSwerveModule().getDriveFeedForward().setKv(driveFFVel);
        m_swerve.getBackRightSwerveModule().getDriveFeedForward().setKa(driveFFAccel);
    }

    public void reportTelemetry() {
        m_xVelEntry.setDouble(m_swerve.getChassisSpeeds().vxMetersPerSecond);
        m_yVelEntry.setDouble(m_swerve.getChassisSpeeds().vyMetersPerSecond);
        m_gyroAngle.setDouble(m_swerve.getGyroYawRotation2d().getDegrees());
        for (SwerveModuleSB sb : mSwerveModuleTelem) {
            sb.update();
        }
        SwerveModuleState[] states = {
                m_swerve.getBackLeftSwerveModule().getState(),
                m_swerve.getBackRightSwerveModule().getState(),
                m_swerve.getFrontLeftSwerveModule().getState(),
                m_swerve.getFrontRightSwerveModule().getState() };
        publisher.set(states);
        m_currentRange.setDouble(m_range.getRange());
        ChassisSpeeds commandedSpeeds = m_swerve.getCommandeChassisSpeeds();
        m_commandedXVel.setDouble(commandedSpeeds.vxMetersPerSecond);
        m_commandedYVel.setDouble(commandedSpeeds.vyMetersPerSecond);
        SmartDashboard.putData("Field", m_swerve.getField2d());
    }
}
