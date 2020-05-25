import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints

object TrajectoryGen {
    private val constraints = DriveConstraints(45.0, 30.0, 0.0, 270.0.toRadians, 270.0.toRadians, 0.0)
    private val startPose = Pose2d(-48.0, -24.0, 170.0.toRadians)

    fun createTrajectory(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        val builder1 = TrajectoryBuilder(Pose2d(-40.0, -63.0, Vector2d(20.0, 31.0).angle()), 0.0, constraints)

        var trajectory1 = builder1
            .strafeTo(Vector2d(-20.0, -32.0))// pick up first block
            .splineToConstantHeading(Pose2d(0.0, -36.0))
            .splineToConstantHeading(Pose2d(54.0, -30.0)) // drop off first block
            .build()

        list.add(trajectory1)

        var builder2 = TrajectoryBuilder(Pose2d(54.0, -30.0, 180.0.toRadians), true, constraints);

        builder2
            .splineToConstantHeading(Pose2d(0.0, -36.0, 180.0.toRadians))
            .splineToConstantHeading(Pose2d(-44.0, -32.5, 180.0.toRadians)) // pick up second block

        list.add(builder2.build())

        var builder3 = TrajectoryBuilder(Pose2d(-44.0, -32.5), false, constraints);

        builder3
            .splineToConstantHeading(Pose2d(0.0, -36.0))
            .splineToConstantHeading(Pose2d(54.0, -30.0)) // drop off second block

        list.add(builder3.build())

        var builder4 = TrajectoryBuilder(Pose2d(54.0, -30.0, 180.0.toRadians), true, constraints);

        builder4
            .lineToLinearHeading(Vector2d(42.0, -36.0), -90.0.toRadians)

        var trajectory4 = builder4.build();
        list.add(trajectory4)

        var builder5 = TrajectoryBuilder(trajectory4.end(), trajectory4.end().heading, constraints);

        builder5
            .lineTo(Vector2d(42.0, -30.0))

        var trajectory5 = builder5.build();
        list.add(trajectory5)

        var builder6 = TrajectoryBuilder(trajectory5.end(), trajectory5.end().heading, constraints);

        builder6
            .lineTo(Vector2d(42.0, -36.0))

        var trajectory6 = builder6.build();
        list.add(trajectory6)

        var builder7 = TrajectoryBuilder(trajectory6.end(), trajectory6.end().heading, constraints);

        builder7
            .lineToLinearHeading(Vector2d(42.0, -50.0), -180.0.toRadians)

        var trajectory7 = builder7.build();
        list.add(trajectory7)

        var builder8 = TrajectoryBuilder(trajectory7.end(), trajectory7.end().heading, constraints);

        builder8
            .lineTo(Vector2d(54.0, -50.0))

        var trajectory8 = builder8.build();
        list.add(trajectory8)

        var builder9 = TrajectoryBuilder(trajectory8.end(), trajectory8.end().heading, constraints);

        builder9
            .splineTo(Pose2d(30.0, -42.0, -270.0.toRadians))
            .splineTo(Pose2d(0.0, -36.0, -180.0.toRadians))

        var trajectory9 = builder9.build();
        list.add(trajectory9)

        return list
    }

    fun drawOffbounds() {
        GraphicsUtil.fillRect(Vector2d(0.0, -63.0), 18.0, 18.0) // robot against the wall
    }
}

val Double.toRadians get() = (Math.toRadians(this))