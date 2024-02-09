/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 // heavily inspired by FRC 6995 NOMAD's 2023 PhotonPoseEstimator.java

package frc.robot.util;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The PhotonPoseEstimator class filters or combines readings from all the AprilTags visible at a
 * given timestamp on the field to produce a single robot in field pose, using the strategy set
 * below.
 */

public class PhotonPoseEstimator {
    /** Position estimation strategies that can be used by the {@link PhotonPoseEstimator} class. */
    // public enum PoseStrategy {
    //     /** Choose the Pose with the lowest ambiguity. */
    //     LOWEST_AMBIGUITY,

    //     /** Choose the Pose which is closest to the camera height. */
    //     CLOSEST_TO_CAMERA_HEIGHT,

    //     /** Choose the Pose which is closest to a set Reference position. */
    //     CLOSEST_TO_REFERENCE_POSE,

    //     /** Choose the Pose which is closest to the last pose calculated */
    //     CLOSEST_TO_LAST_POSE,

    //     /** Return the average of the best target poses using ambiguity as weight. */
    //     AVERAGE_BEST_TARGETS,

    //     /** Use all visible tags to compute a single pose estimate.. */
    //     MULTI_TAG_PNP
    // }

    private AprilTagFieldLayout fieldTags;
    private PoseStrategy primaryStrategy;
    private PoseStrategy multiTagFallBackStrategy = PoseStrategy.LOWEST_AMBIGUITY;
    private PhotonCamera camera;
    private Transform3d robotToCamera;

    private Pose3d lastPose;
    private Pose3d referencePose;
    protected double poseCacheTimestampSeconds = -1;
    private final Set<Integer> reportedErrors = new HashSet<>(); // see what this means idk what it means

    /**
     * Create a new PhotonPoseEstimator.
     *
     * @param fieldTags A WPILib {@link AprilTagFieldLayout} linking AprilTag IDs to Pose3d objects
     *     with respect to the FIRST field using the <a
     *     href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#field-coordinate-system">Field
     *     Coordinate System</a>.
     * @param strategy The strategy it should use to determine the best pose.
     * @param camera PhotonCamera
     * @param robotToCamera Transform3d from the center of the robot to the camera mount position (ie,
     *     robot ➔ camera) in the <a
     *     href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
     *     Coordinate System</a>.
     */

     private PhotonPoseEstimator(AprilTagFieldLayout fieldTags, PoseStrategy poseStrategy, 
        PhotonCamera camera, Transform3d robotToCamera) {
            this.fieldTags = fieldTags;
            this.primaryStrategy = poseStrategy;
            this.camera = camera;
            this.robotToCamera = robotToCamera;
    }

    // invalidates pose cache
    private void invalidatePoseCache() {
        poseCacheTimestampSeconds = -1;
    }

    private void checkUpdate(Object oldObject, Object newObject) {
        if (oldObject != newObject && oldObject != null && !oldObject.equals(newObject)) {
            invalidatePoseCache();
        }
    }

    // get the AprilTagFieldLayout being used by the PositionEstimator
    public AprilTagFieldLayout getFieldTags() {
        return fieldTags;
    }

    // set the AprilTagFieldLayout being used by the PositionEstimator
    public void setFieldTags(AprilTagFieldLayout fieldTags) {
        checkUpdate(this.fieldTags, fieldTags);
        this.fieldTags = fieldTags;
    }

    // get the primary pose strategy being used by the PositionEstimator
    public PoseStrategy getPrimaryStrategy() {
        return primaryStrategy;
    }

    // set the primary pose strategy being used by the PositionEstimator
    public void setPrimaryStrategy(PoseStrategy strategy) {
        checkUpdate(this.primaryStrategy, strategy);
        this.primaryStrategy = strategy;
    }

    // set the Position Estimation Strategy used in multi-tag mode when only one tag can be seen. Can't be MULTI_TAG_PNP
    public void setMultiTagFallbackStrategy(PoseStrategy strategy) {
        if (strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) {
            DriverStation.reportWarning(
                "Fallback cannot be set to MULTI_TAG_PNP! Now setting to lowest ambiguity", null);
            strategy = PoseStrategy.LOWEST_AMBIGUITY;
        }
        this.multiTagFallBackStrategy = strategy;
    }

    // get the reference position being used by the estimator
    public Pose3d getReferencePose() {
        return referencePose;
    }

    // set the stored reference pose to an updated Pose3d value for use with <b>CLOSEST_TO_REFERENCE_Pose<b> strategy
    public void setReferencePose(Pose3d referencePose) {
        checkUpdate(this.referencePose, referencePose);
        this.referencePose = referencePose;
    }

    // set the stored reference pose to an updated Pose2d value for use with <b>CLOSEST_TO_REFERENCE_Pose<b> strategy
    public void setReferencePose(Pose2d referencePose) {
        setReferencePose(new Pose3d(referencePose));
    }

    // update the stored last pose with Pose3d parameter
    public void setLastPose(Pose3d lastPose) {
        this.lastPose = lastPose;
    }

    // update the stored last pose with Pose2d parameter
    public void setLastPose(Pose2d lastPose) {
        setLastPose(new Pose3d(lastPose));
    }

    // get the currect transform from center of robot ot the camera mount position
    public Transform3d getRobotToCameraTransform() {
        return robotToCamera;
    }

    // set the current transform from the center of the robot to the camera mount position
    public void setRobotToCameraTansform(Transform3d robotToCamera) {
        this.robotToCamera = robotToCamera;
    }

    private Optional<EstimatedRobotPose> multiTagPNPStrategy(PhotonPipelineResult result) {
        var visCorners = new ArrayList<TargetCorner>();
        // List<PhotonTrackedTarget> visTags = new ArrayList<PhotonTrackedTarget>();
        var knownVisTags = new ArrayList<AprilTag>();
        var fieldToCams = new ArrayList<Pose3d>();
        var fieldToCamsAlt = new ArrayList<Pose3d>();
        List<Integer> fiducialIDsUsed = new ArrayList<Integer>();

        if (result.getTargets().size() < 2) {
            // run fallback strategy
            return update(result, this.multiTagFallBackStrategy);
        }

        for (var target : result.getTargets()) {
            visCorners.addAll(target.getDetectedCorners());
            fiducialIDsUsed.add(target.getFiducialId());
            var tagPoseOpt = fieldTags.getTagPose(target.getFiducialId());
            if (tagPoseOpt.isEmpty()) {
                reportFiducialPoseError(target.getFiducialId());
                continue;
            }

            var tagPose = tagPoseOpt.get();

            knownVisTags.add(new AprilTag(target.getFiducialId(), tagPose));

            fieldToCams.add(tagPose.transformBy(target.getBestCameraToTarget().inverse()));
            fieldToCamsAlt.add(tagPose.transformBy(target.getAlternateCameraToTarget().inverse()));
        }

        var cameraMatrixOpt = camera.getCameraMatrix();
        var distCoeffsOpt = camera.getDistCoeffs();
        boolean hasCalibData = cameraMatrixOpt.isPresent() && distCoeffsOpt.isPresent();

        if (hasCalibData) {
            var cameraMatrix = cameraMatrixOpt.get();
            var distCoeffs = distCoeffsOpt.get();
            // var pnpResults = VisionEstimation.estimateCamPosePNP(cameraMatrix, distCoeffs, visCorners, fieldTags, null);
            // var pnpResults = new MultiTargetPNPResult(null, fiducialIDsUsed);
            var pnpResults =
                    VisionEstimation.estimateCamPosePNP(cameraMatrix, distCoeffs, visCorners, knownVisTags);
            var best = new Pose3d().plus(pnpResults.best).plus(robotToCamera.inverse());
        }
    }

    private void reportFiducialPoseError(int fiducialId) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'reportFiducialPoseError'");
    }

    //
    public Optional<EstimatedRobotPose> update() {
        if (camera == null) {
            DriverStation.reportError("Error: [PhotonPoseEstimator] Missing camera!", false);
            return Optional.empty();
        }

        PhotonPipelineResult cameraResult = camera.getLatestResult();
        return update(cameraResult);
    }

    //
    public Optional<EstimatedRobotPose> update(PhotonPipelineResult cameraResult) {
        if (cameraResult.getTimestampSeconds() < 0) {
            return Optional.empty();
        }

        if (poseCacheTimestampSeconds > 0 && Math.abs(poseCacheTimestampSeconds - cameraResult.getTimestampSeconds()) < 1e-6) {
            return Optional.empty();
        }

        return update(cameraResult, this.primaryStrategy);
    }

    //

    private Optional<EstimatedRobotPose> update(PhotonPipelineResult cameraResult, PoseStrategy strat) {
        Optional<EstimatedRobotPose> estimatedPose;
        switch(strat) {
            case LOWEST_AMBIGUITY:
                estimatedPose = lowestAmbiguityStrategy(cameraResult);
                break;
            case CLOSEST_TO_CAMERA_HEIGHT:
                estimatedPose = closestToCameraHeightStrategy(cameraResult);
                break;
        }

        if (estimatedPose.isEmpty()) {
            lastPose = null;
        }

        return estimatedPose;
    }


}
