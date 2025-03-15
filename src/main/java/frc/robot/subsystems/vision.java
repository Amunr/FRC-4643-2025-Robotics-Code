    package frc.robot.subsystems;

    import java.lang.StackWalker.Option;
    import java.util.List;
    import java.util.Optional;

    import org.photonvision.EstimatedRobotPose;
    import org.photonvision.PhotonCamera;
    import org.photonvision.PhotonPoseEstimator;
    import org.photonvision.PhotonPoseEstimator.PoseStrategy;
    import org.photonvision.targeting.PhotonPipelineResult;
    import org.photonvision.targeting.PhotonTrackedTarget;

    import edu.wpi.first.apriltag.AprilTagFieldLayout;
    import edu.wpi.first.apriltag.AprilTagFields;
    import edu.wpi.first.math.geometry.Pose2d;
    import edu.wpi.first.math.geometry.Rotation3d;
    import edu.wpi.first.math.geometry.Transform3d;
    import edu.wpi.first.math.geometry.Translation3d;
    import edu.wpi.first.wpilibj2.command.SubsystemBase;
    import swervelib.SwerveDrive;
    import swervelib.telemetry.SwerveDriveTelemetry;

    public class Vision extends SubsystemBase {
        private Optional<AprilTagFieldLayout> optionalFieldLayout;
        private AprilTagFieldLayout aprilTagFieldLayout;
        private PhotonCamera camera;
        private Transform3d robotToCam;
        private PhotonPoseEstimator photonPoseEstimator;
        private List<PhotonPipelineResult> unreadResults;
        private PhotonPipelineResult latestResult;

        public Vision() {
            AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

            camera = new PhotonCamera("photonvision");
            robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)); // Cam mounted
                                                                                                    // facing // forward,
                                                                                                    // half a meter
                                                                                                    // forward of
                                                                                                    // center, half
                                                                                                    // a meter up
                                                                                                    // from center.
            photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        }

        // Construct PhotonPoseEstimator
        public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
            try {
                photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
                unreadResults = camera.getAllUnreadResults();
                if (unreadResults.isEmpty()) {
                    return Optional.empty();
                }
                latestResult = unreadResults.get(unreadResults.size() - 1);
                if (latestResult == null) {
                    return Optional.empty(); // Avoid null pointer issues
                }

                return photonPoseEstimator.update(latestResult);
            } catch (Exception e) {
                System.err.println("Error in pose generation");
                return Optional.empty();
            }
        }

        public void updatePoseEstimation(SwerveDrive swerveDrive, Pose2d prevEstimatedRobotPose) {
            Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(prevEstimatedRobotPose);
            
            poseEst.ifPresent(pose -> {
                try {
                    swerveDrive.addVisionMeasurement(
                        pose.estimatedPose.toPose2d(),  // Convert estimated pose to Pose2d
                        pose.timestampSeconds          // Timestamp from the vision system
                    );
                } catch (Exception e) {
                    System.err.println("Error adding vision measurement: " + e.getMessage());
                }
            });
        
    }
    }
