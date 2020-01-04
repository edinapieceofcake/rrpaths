import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints

object TrajectoryGen {
    private val constraints = DriveConstraints(45.0, 30.0, 0.0, 270.0.toRadians, 270.0.toRadians, 0.0)
    private val startPose = Pose2d(-48.0, -24.0, 170.0.toRadians)

    fun createTrajectory(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        val builder1 = TrajectoryBuilder(Pose2d(-40.0, -63.0, Math.toRadians(0.0)), constraints)
        val builder2 = TrajectoryBuilder(Pose2d(-40.0, -63.0, Math.toRadians(0.0)), constraints)
        val builder3 = TrajectoryBuilder(Pose2d(-40.0, -63.0, Math.toRadians(0.0)), constraints)

        // dump routine
        builder1
            .strafeTo(Vector2d(-22.0, -32.0))// pick up first block
            .lineTo(Vector2d(0.0, -36.0))
            .splineTo(Pose2d(55.0, -30.0)) // drop off first block
            .reverse() // drive backwards
            .splineTo(Pose2d(0.0, -36.0))
            .lineTo(Vector2d(-46.0, -32.0)) // pick up second block
            .reverse()
            .splineTo(Pose2d(0.0, -36.0))
            .splineTo(Pose2d(50.0, -30.0)) // drop off second block
            .reverse() // drive backwards
            .strafeTo(Vector2d(42.0, -36.0)) // turn

        builder2
            .strafeTo(Vector2d(-30.0, -32.0)) // pick up first block
            .lineTo(Vector2d(0.0, -36.0))
            .splineTo(Pose2d(55.0, -30.0)) // drop off first block
            .reverse() // drive backwards
            .splineTo(Pose2d(0.0, -36.0))
            .lineTo(Vector2d(-54.0, -32.0)) // pick up second block
            .reverse()
            .splineTo(Pose2d(0.0, -36.0))
            .splineTo(Pose2d(50.0, -30.0)) // drop off second block
            .reverse() // drive backwards
            .strafeTo(Vector2d(42.0, -36.0)) // turn

        builder3
            .strafeTo(Vector2d(-38.0, -32.0)) // pick up first block
            .lineTo(Vector2d(0.0, -36.0))
            .splineTo(Pose2d(55.0, -30.0)) // drop off first block
            .reverse() // drive backwards
            .splineTo(Pose2d(0.0, -36.0))
            .lineTo(Vector2d(-62.0, -32.0)) // pick up second block
            .reverse()
            .splineTo(Pose2d(0.0, -36.0))
            .splineTo(Pose2d(50.0, -30.0)) // drop off second block
            .reverse() // drive backwards
            .strafeTo(Vector2d(42.0, -36.0)) // turn

        list.add(builder1.build())
        list.add(builder2.build())
        list.add(builder3.build())
        return list
    }

    fun drawOffbounds() {
        GraphicsUtil.fillRect(Vector2d(0.0, -63.0), 18.0, 18.0) // robot against the wall
    }
}

val Double.toRadians get() = (Math.toRadians(this))