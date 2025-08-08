package frc.robot.subsystems

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.net.PortForwarder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.RobotParameters.PhotonVisionConstants.CAMERA_ONE_HEIGHT_METER
import frc.robot.utils.getDecentResultPairs
import frc.robot.utils.hasTargets
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import xyz.malefic.frc.pingu.LogPingu.log
import xyz.malefic.frc.pingu.LogPingu.logs

/**
 * The PhotonVision class is a subsystem that interfaces with multiple PhotonVision cameras to
 * provide vision tracking and pose estimation capabilities. This subsystem is a Singleton that
 * manages multiple CameraModules and selects the best result based on pose ambiguity.
 *
 * This subsystem provides methods to get the estimated global pose of the robot, the distance to
 * targets, and the yaw of detected AprilTags. It also provides methods to check if a tag is visible
 * and get the pivot position based on distance calculations.
 */
object PhotonVision : SubsystemBase() {
    private val cameras: MutableList<PhotonModule> = mutableListOf()
    private var yaw: Double = 0.0
    private var y: Double = 0.0
    private var dist: Double = 0.0
    private var logCount: Int = 0
    private var currentResultPair: List<Pair<PhotonModule, PhotonPipelineResult>> = emptyList()

    init {
        cameras.add(
            PhotonModule(
                "RightCamera",
                Transform3d(
                    Translation3d(0.27305, -0.2985, CAMERA_ONE_HEIGHT_METER),
                    Rotation3d(0.0, Math.toRadians(-25.0), Math.toRadians(45.0)),
                ),
                AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
            ),
        )
        // well calibrated camera is left camera
        cameras.add(
            PhotonModule(
                "LeftCamera",
                Transform3d(
                    Translation3d(0.27305, 0.2985, CAMERA_ONE_HEIGHT_METER),
                    Rotation3d(0.0, Math.toRadians(-25.0), Math.toRadians(-45.0)),
                ),
                AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
            ),
        )
        currentResultPair = emptyList()
        PortForwarder.add(5800, "photonvision.local", 5800)
    }

    override fun periodic() {
        currentResultPair = cameras.getDecentResultPairs()

        logs {
            log("Photonvision/Does any camera exist", cameras.getOrNull(0) != null)
            log("Photonvision/Has tag", hasTag())
            log("Photonvision/resultCamera List length", currentResultPair.size)
        }

        logs("Photonvision/Best target list is empty", currentResultPair.isEmpty())
        if (currentResultPair.isNotEmpty()) {
            logCount++
            logs("Photonvision/BestTarget updated counter", logCount)
            val bestTarget: PhotonTrackedTarget? = currentResultPair[0].second.bestTarget
            logs("Photonvision/BestTarget is not null", bestTarget != null)
            logs("Photonvision/Best Target is not null", bestTarget != null)
            logs("Photonvision/Best Target is not null", bestTarget != null)
            bestTarget?.let {
                yaw = it.yaw
                y = it.bestCameraToTarget.x
                dist = it.bestCameraToTarget.z
            }
            // The original code repeats this block - keep for parity
            bestTarget?.let {
                yaw = it.yaw
                y = it.bestCameraToTarget.x
                dist = it.bestCameraToTarget.z
            }
            logs("Yaw", yaw)
        }
        logStdDev()
    }

    /**
     * Checks if there is a visible tag.
     *
     * @return true if there is a visible tag and the current result pair is not null
     */
    fun hasTag(): Boolean {
        logs("Photonvision/hasTargets currentResultPair", currentResultPair.hasTargets())
        return currentResultPair.hasTargets()
    }

    /**
     * Gets the current yaw angle to the target.
     *
     * @return The yaw angle in degrees
     */
    fun getYaw(): Double = yaw

    /**
     * Gets the current distance to the target.
     *
     * @return The distance in meters
     */
    fun getDist(): Double = dist

    /**
     * Gets the current Y position of the target.
     *
     * @return The Y position in meters
     */
    fun getY(): Double = y

    fun requestCamera(cameraName: String): PhotonCamera? {
        for (camera in cameras) {
            if (camera.cameraName == cameraName) {
                return camera.camera
            }
        }
        return null
    }

    fun fetchYaw(camera: PhotonCamera): Double {
        for (pair in currentResultPair) {
            if (pair.first.camera == camera) {
                return pair.second.bestTarget.yaw
            }
        }
        return 0.0
    }

    fun fetchDist(camera: PhotonCamera): Double {
        for (pair in currentResultPair) {
            if (pair.first.camera == camera) {
                return pair.second.bestTarget.bestCameraToTarget.x
            }
        }
        return 0.0
    }

    fun fetchY(camera: PhotonCamera): Double {
        for (pair in currentResultPair) {
            if (pair.first.camera == camera) {
                return pair.second.bestTarget.bestCameraToTarget.y
            }
        }
        return 7157.0
    }

    /**
     * Logs the standard deviation norm for each camera. This method filters out cameras with null
     * standard deviations and logs the normF value of the standard deviations for each camera.
     */
    fun logStdDev() {
        cameras
            .filter { it.currentStdDevs != null }
            .forEach { camera ->
                logs(
                    "Photonvision/Camera %s Std Dev NormF".format(camera.cameraName),
                    camera.currentStdDevs?.normF() ?: 0.0,
                )
            }
    }

    fun getResultPairs(): List<Pair<PhotonModule, PhotonPipelineResult>> = currentResultPair
}
