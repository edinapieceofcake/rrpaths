import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator
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
            .splineTo(Pose2d(0.0, -36.0))
            .splineTo(Pose2d(55.0, -30.0)) // drop off first block
            .reverse() // drive backwards
            .splineTo(Pose2d(0.0, -36.0))
            .splineTo(Pose2d(-46.0, -32.0)) // pick up second block
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

        val builder4 = TrajectoryBuilder(Pose2d(-62.0, -32.0, Math.toRadians(0.0)), constraints)
        builder4
            .splineTo(Pose2d(0.0, -36.0))
            .splineTo(Pose2d(50.0, -30.0)) // drop off second block
            .reverse()
            .lineTo(Vector2d(42.0, -36.0), LinearInterpolator(-90.0.toRadians, 90.0.toRadians))
            .lineTo(Vector2d(42.0, -30.0))
            .reverse()
            .lineTo(Vector2d(38.0, -53.0), LinearInterpolator(-90.0.toRadians, -90.0.toRadians))
            //.lineTo(Vector2d(8.0, -34.0))
            //.splineTo(Pose2d(8.0, -34.0), LinearInterpolator(-90.0.toRadians, -180.0.toRadians)) // drive to bridge

//        list.add(builder1.build())
//        list.add(builder2.build())
//        list.add(builder3.build())
//        list.add(builder4.build())

        val builder5 = TrajectoryBuilder(Pose2d(-40.0, -63.0, Math.toRadians(0.0)), constraints)
        builder5
            .strafeTo(Vector2d(-22.0, -32.0))// pick up first block
            .lineTo(Vector2d(0.0, -36.0))
            .splineTo(Pose2d(30.0, -30.0)) // drop off first block
            .reverse() // drive backwards
            .splineTo(Pose2d(0.0, -36.0))
            .lineTo(Vector2d(-46.0, -32.0)) // pick up second block
            .reverse()
            .splineTo(Pose2d(0.0, -36.0))
            .strafeTo(Vector2d(25.0, -30.0)) // drop off second block
            .reverse() // drive backwards
            .strafeTo(Vector2d(0.0, -36.0))
            .splineTo(Pose2d(-30.0, -32.0)) // pick up second block
            .reverse()
            .splineTo(Pose2d(0.0, -36.0))
            .strafeTo(Vector2d(20.0, -30.0)) // drop off first block
            .reverse()
            .strafeTo(Vector2d(0.0, -36.0))

//        list.add(builder5.build())

        val builder6 = TrajectoryBuilder(Pose2d(38.0, -53.0, Math.toRadians(-180.0)), constraints)
        builder6
            .splineTo(Pose2d(0.0, -36.0, Math.toRadians(-180.0)))


//        list.add(builder6.build())

        val builder7 = TrajectoryBuilder(Pose2d(15.0, -63.0, Math.toRadians(0.0)), constraints)
        builder7
            .reverse()
            .lineTo(Vector2d(0.0, -63.0))

//        list.add(builder7.build())

        val builder8 = TrajectoryBuilder(Pose2d(40.0, -63.0, Math.toRadians(-90.0)), constraints)
        builder8
            .reverse()
            .splineTo(Pose2d(50.0, -30.0, Math.toRadians(-90.0)))
            .reverse()
            .lineTo(Vector2d(50.0, -61.0))
            .strafeTo(Vector2d(-8.0, -62.0))

//        list.add(builder8.build())

        val builder9 = TrajectoryBuilder(Pose2d(40.0, 63.0, Math.toRadians(90.0)), constraints)
        builder9
            .reverse()
            .splineTo(Pose2d(50.0, 30.0, Math.toRadians(90.0)))
            .reverse()
            .lineTo(Vector2d(50.0, 61.0))
            .strafeTo(Vector2d(-8.0, 62.0))

//        list.add(builder9.build())


        val builder10 = TrajectoryBuilder(Pose2d(-40.0, -63.0, Math.toRadians(0.0)), constraints)
        builder10
            .strafeTo(Vector2d(-22.0, -32.0))// pick up first block
            .splineTo(Pose2d(0.0, -36.0))
            .splineTo(Pose2d(55.0, -30.0)) // drop off first block
            .reverse() // drive backwards
            .splineTo(Pose2d(0.0, -36.0))
            .splineTo(Pose2d(-46.0, -32.0)) // pick up second block
            .reverse()
            .splineTo(Pose2d(0.0, -36.0))
            .splineTo(Pose2d(50.0, -30.0)) // drop off second block
            .reverse() // drive backwards
            .splineTo(Pose2d(0.0, -38.0))
            .splineTo(Pose2d(-50.0, -20.0), LinearInterpolator(Math.toRadians(180.0), Math.toRadians(0.0)))
            .reverse()
            .splineTo(Pose2d(0.0, -38.0))

//        list.add(builder10.build());

        val builder11 = TrajectoryBuilder(Pose2d(42.0, -53.0, Math.toRadians(-180.0)), constraints)
        builder11
            .splineTo(Pose2d(26.0, -48.0, Math.toRadians(90.0)))
            .splineTo(Pose2d(0.0, -38.0, Math.toRadians(-180.0)))

        list.add(builder11.build());

        return list
    }

    fun drawOffbounds() {
        GraphicsUtil.fillRect(Vector2d(0.0, -63.0), 18.0, 18.0) // robot against the wall
    }
}

val Double.toRadians get() = (Math.toRadians(this))