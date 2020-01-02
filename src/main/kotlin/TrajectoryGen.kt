import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints

object TrajectoryGen {
    private val constraints = DriveConstraints(45.0, 30.0, 0.0, 270.0.toRadians, 270.0.toRadians, 0.0)
    private val startPose = Pose2d(-48.0, -24.0, 170.0.toRadians)

    fun createTrajectory(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        val builder1 = TrajectoryBuilder(Pose2d(-40.0, -63.0, 0.0.toRadians), constraints)
        val builder2 = TrajectoryBuilder(Pose2d(-40.0, 63.0, 0.0.toRadians), constraints)

        // dump routine
        builder1
            .strafeTo(Vector2d(-22.0, -32.0)) // pick up first block
            .lineTo(Vector2d(0.0, -40.0))
            .splineTo(Pose2d(60.0, -30.0)) // drop off first block
            .reverse() // drive backwards
            .splineTo(Pose2d(0.0, -40.0))
            .lineTo(Vector2d(-46.0, -32.0)) // pick up second block
            .reverse() // drive forwards
            .splineTo(Pose2d(0.0, -40.0))
            .splineTo(Pose2d(50.0, -30.0)) // drop off second block
            .reverse() // drive backwards
            .lineTo(Vector2d(42.0, -34.0), ConstantInterpolator(-90.0.toRadians)) // turn
            .lineTo(Vector2d(42.0, -30.0), ConstantInterpolator(-90.0.toRadians)) // backup
            .lineTo(Vector2d(42.0, -54.0), ConstantInterpolator(-180.0.toRadians)) // drag forward and turn
            .lineTo(Vector2d(0.0, -40.0), ConstantInterpolator(-180.0.toRadians))

        builder2
            .strafeTo(Vector2d(-22.0, 32.0)) // pick up first block
            .lineTo(Vector2d(0.0, 40.0))
            .splineTo(Pose2d(60.0, 30.0)) // drop off first block
            .reverse() // drive backwards
            .splineTo(Pose2d(0.0, 40.0))
            .lineTo(Vector2d(-46.0, 32.0)) // pick up second block
            .reverse() // drive forwards
            .splineTo(Pose2d(0.0, 40.0))
            .splineTo(Pose2d(50.0, 30.0)) // drop off second block
            .reverse() // drive backwards
            .lineTo(Vector2d(42.0, 34.0), ConstantInterpolator(90.0.toRadians)) // turn
            .lineTo(Vector2d(42.0, 30.0), ConstantInterpolator(90.0.toRadians)) // backup
            .lineTo(Vector2d(42.0, 54.0), ConstantInterpolator(180.0.toRadians)) // drag forward and turn
            .lineTo(Vector2d(0.0, 40.0), ConstantInterpolator(180.0.toRadians))


        list.add(builder2.build())
        list.add(builder1.build())
        return list
    }

    fun drawOffbounds() {
        GraphicsUtil.fillRect(Vector2d(0.0, -63.0), 18.0, 18.0) // robot against the wall
    }
}

val Double.toRadians get() = (Math.toRadians(this))