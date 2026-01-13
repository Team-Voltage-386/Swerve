package frc.robot.Commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SparkJrConstants;
//import frc.robot.TyRap24Constants.*;
import frc.robot.SparkJrConstants.*;
import frc.robot.Subsystems.Drivetrain;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PhotonDrive extends Command {
    Drivetrain dt;
    private final XboxController m_controller = new XboxController(Controller.kDriveControllerID);
    double forward;
    double strafe;
    double turn;
    private final NetworkTable table;
    private boolean targetVisible;
    private double targetYaw;
    private double targetRange;
    private final PhotonCamera camera = new PhotonCamera("SparkJrCam");
    private final DoubleEntry m_targetYaw;
    private final DoubleEntry m_targetHeight;
    private final DoubleEntry m_turnkP;
    private final DoubleEntry m_strafekP;
    private final DoubleEntry m_desiredAngle;
    private final DoubleEntry m_desiredRange;
    private final IntegerEntry m_targetid;
    private final DoubleEntry m_cameraPitch;
    private final DoubleEntry m_cameraHeight;

    public PhotonDrive(Drivetrain dt, NetworkTableInstance nt) {
        this.dt = dt;
        table = nt.getTable(getName());
        m_targetYaw = table.getDoubleTopic("Target Yaw").getEntry(targetYaw);
        m_targetHeight = table.getDoubleTopic("Target Range").getEntry(targetRange);
        m_turnkP = table.getDoubleTopic("Turn kP").getEntry(PhotonConstants.VISION_TURN_kP);
        m_strafekP = table.getDoubleTopic("Strafe kP").getEntry(PhotonConstants.VISION_STRAFE_kP);
        m_desiredAngle = table.getDoubleTopic("Desired Angle").getEntry(PhotonConstants.VISION_DES_ANGLE_deg);
        m_desiredRange = table.getDoubleTopic("Desired Range").getEntry(PhotonConstants.VISION_DES_RANGE_m);
        m_targetid = table.getIntegerTopic("Target ID").getEntry(7);
        m_cameraPitch = table.getDoubleTopic("Camera Pitch").getEntry(PhotonConstants.kCameraPitchDegrees);
        m_cameraHeight = table.getDoubleTopic("Camera Height").getEntry(PhotonConstants.kCameraHeightMeters);


        m_turnkP.set(PhotonConstants.VISION_TURN_kP);
        m_strafekP.set(PhotonConstants.VISION_STRAFE_kP);     
        m_desiredAngle.set(PhotonConstants.VISION_DES_ANGLE_deg);
        m_desiredRange.set(PhotonConstants.VISION_DES_RANGE_m);
        m_targetid.set(7);
        m_cameraPitch.set(PhotonConstants.kCameraPitchDegrees);
        m_cameraHeight.set(PhotonConstants.kCameraHeightMeters);
        addRequirements(dt);
    }
    @Override
    public void execute() {
        // Read in relevant data from the Camera
        forward = -m_controller.getLeftY() * 3;
        strafe = -m_controller.getLeftX() * 3;
        turn = -m_controller.getRightX() * 4.7;
        targetVisible = false;
        targetYaw = 0.0;
        targetRange = 0.0;
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == m_targetid.get()) {
                        // Found Tag 7, record its information
                        targetYaw = target.getYaw();
                        targetRange =
                                PhotonUtils.calculateDistanceToTargetMeters(
                                        m_cameraHeight.get(), // Measured with a tape measure, or in CAD.
                                        m_targetHeight.get(), // From 2024 game manual for ID 7
                                        Units.degreesToRadians(m_cameraPitch.get()), // Measured with a protractor, or in CAD.
                                        Units.degreesToRadians(target.getPitch()));

                        targetVisible = true;
                    }
                }
            }
        }

        // Auto-align when requested
        if (targetVisible) {
            // Driver wants auto-alignment to tag 7
            // And, tag 7 is in sight, so we can turn toward it.
            // Override the driver's turn and fwd/rev command with an automatic one
            // That turns toward the tag, and gets the range right.
            turn = (m_desiredAngle.get() - targetYaw) * PhotonConstants.VISION_TURN_kP * 4.7;
            forward = (m_desiredRange.get() - targetRange) * PhotonConstants.VISION_STRAFE_kP * 3;
        }

        // Command drivetrain motors based on target speeds
        dt.drive(forward, strafe, turn);
        m_targetYaw.set(targetYaw);
        m_targetHeight.set(targetRange);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
