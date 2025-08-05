package frc.robot.subsystems

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.numbers.N4
import frc.robot.utils.RobotParameters.PhotonVisionConstants
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.Optional
import kotlin.math.sqrt

/**
 * The CameraModule class represents a single Photonvision camera setup with its associated pose
 * estimator and position information. This class encapsulates all the functionality needed for a
 * single camera to track AprilTags and estimate robot pose.
 */
class PhotonModule(
    cameraName: String?,
    cameraPos: Transform3d?,
    fieldLayout: AprilTagFieldLayout?,
) {
    val camera: PhotonCamera = PhotonCamera(cameraName)

    /**
     * Gets the pose estimator associated with this camera.
     *
     * @return PhotonPoseEstimator, The PhotonPoseEstimator object used for robot pose estimation
     */
    val poseEstimator: PhotonPoseEstimator =
        PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraPos).apply {
            setMultiTagFallbackStrategy(
                PoseStrategy.LOWEST_AMBIGUITY,
            )
        }

    /**
     * Gets the camera's position relative to the robot.
     *
     * @return [Transform3d], The [Transform3d] representing the camera's position
     */
    val cameraPosition: Transform3d? = cameraPos

    /**
     * Gets the current standard deviations used for robot pose estimation.
     *
     * @return Matrix<N3></N3>, N1> The current standard deviations as a Matrix object
     */
    var currentStdDevs: Matrix<N3, N1>? = null
        private set

    /**
     * Gets the current standard deviations used for 3D robot pose estimation.
     *
     * @return Matrix<N4></N4>, N1> The current standard deviations as a Matrix object
     */
    var currentStdDevs3d: Matrix<N4, N1>? = null
        private set

    val allUnreadResults: MutableList<PhotonPipelineResult?>
        /**
         * Gets all unread pipeline results from the camera.
         *
         * @return A list of PhotonPipelineResult objects containing the latest vision processing results
         */
        get() = camera.getAllUnreadResults()

    /**
     * Updates the estimated standard deviations based on the provided estimated pose and list of
     * tracked targets.
     *
     *
     * This method calculates the number of visible tags and their average distance to the
     * estimated pose. It then uses this information to adjust the standard deviations used for robot
     * pose estimation.
     *
     * @param estimatedPose An Optional containing the estimated robot pose.
     * @param targets A list of PhotonTrackedTarget objects representing the tracked targets.
     */
    fun updateEstimatedStdDevs(
        estimatedPose: Optional<EstimatedRobotPose>,
        targets: MutableList<PhotonTrackedTarget>,
    ) {
        if (estimatedPose.isEmpty) {
            this.currentStdDevs = PhotonVisionConstants.SINGLE_TARGET_STD_DEV
            return
        }
        var numTags = 0
        var totalDistance = 0.0

        // Calculate the number of visible tags and their average distance to the estimated pose
        for (target in targets) {
            val tagPoseOptional = poseEstimator.fieldTags.getTagPose(target.getFiducialId())
            if (tagPoseOptional.isEmpty) continue

            numTags++
            val tagPose = tagPoseOptional.get().toPose2d().translation
            val estimatedTranslation =
                estimatedPose
                    .get()
                    .estimatedPose
                    .toPose2d()
                    .translation
            totalDistance += tagPose.getDistance(estimatedTranslation)
        }

        if (numTags == 0) {
            this.currentStdDevs = PhotonVisionConstants.SINGLE_TARGET_STD_DEV
            return
        }

        val avgDistance = totalDistance / numTags
        val stdDevs: Matrix<N3, N1> =
            if (numTags > 1) {
                PhotonVisionConstants.MULTI_TARGET_STD_DEV
            } else {
                PhotonVisionConstants.SINGLE_TARGET_STD_DEV
            }

        if (numTags == 1 && avgDistance > 4) {
            this.currentStdDevs =
                VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE)
        } else {
            this.currentStdDevs = stdDevs.times(1 + (avgDistance * avgDistance / 30))
        }
    }

    /**
     * Updates the estimated standard deviations based on the provided estimated pose and list of
     * tracked targets. This method calculates the number of visible tags and their average 3D
     * distance to the estimated pose. It then uses this information to adjust the standard deviations
     * used for robot pose estimation.
     *
     * @param estimatedPose
     * @param targets
     */
    fun updateEstimatedStdDevs3d(
        estimatedPose: Optional<EstimatedRobotPose>,
        targets: MutableList<PhotonTrackedTarget>,
    ) {
        if (estimatedPose.isEmpty) {
            this.currentStdDevs3d = PhotonVisionConstants.SINGLE_TARGET_STD_DEV_3D
            return
        }
        var numTags = 0
        var totalDistance = 0.0

        // Calculate the number of visible tags and their average 3D distance to the estimated pose
        for (target in targets) {
            val tagPoseOptional = poseEstimator.fieldTags.getTagPose(target.getFiducialId())
            if (tagPoseOptional.isEmpty) continue

            numTags++
            val tagPose = tagPoseOptional.get().translation
            val estimatedTranslation = estimatedPose.get().estimatedPose.translation

            // Calculate 3D Euclidean distance
            val deltaX = tagPose.x - estimatedTranslation.x
            val deltaY = tagPose.y - estimatedTranslation.y
            val deltaZ = tagPose.z - estimatedTranslation.z
            totalDistance += sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ)
        }

        if (numTags == 0) {
            this.currentStdDevs3d = PhotonVisionConstants.SINGLE_TARGET_STD_DEV_3D
            return
        }

        val avgDistance = totalDistance / numTags
        val stdDevs: Matrix<N4, N1> =
            if (numTags > 1) {
                PhotonVisionConstants.MULTI_TARGET_STD_DEV_3D
            } else {
                PhotonVisionConstants.SINGLE_TARGET_STD_DEV_3D
            }

        if (numTags == 1 && avgDistance > 4) {
            this.currentStdDevs3d =
                VecBuilder.fill(
                    Double.MAX_VALUE,
                    Double.MAX_VALUE,
                    Double.MAX_VALUE,
                    Double.MAX_VALUE,
                )
        } else {
            this.currentStdDevs3d = stdDevs.times(1 + (avgDistance * avgDistance / 30))
        }
    }

    val cameraName: String?
        /**
         * Gets the name of the camera associated with this module.
         *
         * @return String The name of the camera
         */
        get() = camera.name
}
