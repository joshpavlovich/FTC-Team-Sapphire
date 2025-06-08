package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.PathChain
import com.pedropathing.pathgen.Point
import com.pedropathing.util.Constants
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robot.ArmState
import org.firstinspires.ftc.teamcode.robot.BucketState
import org.firstinspires.ftc.teamcode.robot.Robot
import pedroPathing.constants.FConstants
import pedroPathing.constants.LConstants

@Autonomous(name = "Specimen To Observation Zone Auto", group = "Robot")
class SpecimenToObservationZoneAutonomous : LinearOpMode() {

    // Instance of the "Robot" class
    private val robot = Robot(this)

    private val follower: Follower by lazy {
        Constants.setConstants(FConstants::class.java, LConstants::class.java)
        Follower(hardwareMap, FConstants::class.java, LConstants::class.java)
    }

    private lateinit var pathTimer: Timer

    /**
     * This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method.
     */
    enum class PathState {
        START_WITH_PRELOAD,
        PRE_PUSH_SAMPLE_1,
        PUSH_SAMPLE_1_TO_OBSERVATION_ZONE,
        PRE_PUSH_SAMPLE_2,
        PUSH_SAMPLE_2_TO_OBSERVATION_ZONE,
        PRE_PUSH_SAMPLE_3,
        PUSH_SAMPLE_3_TO_OBSERVATION_ZONE,
        PRE_END_GAME_PARK_OBSERVATION_ZONE,
        END_GAME_PARK_OBSERVATION_ZONE
    }

    private var pathState = PathState.START_WITH_PRELOAD
        set(value) {
            field = value
            pathTimer.resetTimer()
        }

    private val startPose = Pose(8.0, 56.0, Math.toRadians(90.0)) // Starting position
    private var prePushSample1: PathChain? = null
    private var pushSample1ToObservationZone: PathChain? = null
    private var prePushSample2: PathChain? = null
    private var pushSample2ToObservationZone: PathChain? = null
    private var prePushSample3: PathChain? = null
    private var pushSample3ToObservationZone: PathChain? = null
    private var preEndGameParkObservationZone: PathChain? = null
    private var endGameParkObservationZone: PathChain? = null

    fun buildPaths() {
        prePushSample1 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(8.000, 56.000, Point.CARTESIAN),
                    Point(41.000, 27.000, Point.CARTESIAN),
                    Point(70.000, 40.000, Point.CARTESIAN),
                    Point(62.500, 23.000, Point.CARTESIAN)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(90.0))
            .build()

        pushSample1ToObservationZone = follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(62.500, 23.000, Point.CARTESIAN),
                    Point(13.000, 23.000, Point.CARTESIAN)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(90.0))
            .build()

        prePushSample2 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(13.000, 23.000, Point.CARTESIAN),
                    Point(60.000, 37.000, Point.CARTESIAN),
                    Point(62.500, 13.000, Point.CARTESIAN)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(90.0))
            .build()

        pushSample2ToObservationZone = follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(62.500, 13.000, Point.CARTESIAN),
                    Point(13.000, 13.000, Point.CARTESIAN)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(90.0))
            .build()

        prePushSample3 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(13.000, 13.000, Point.CARTESIAN),
                    Point(60.000, 19.000, Point.CARTESIAN),
                    Point(62.500, 9.000, Point.CARTESIAN)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(90.0))
            .build()

        pushSample3ToObservationZone = follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(62.500, 9.000, Point.CARTESIAN),
                    Point(13.000, 9.000, Point.CARTESIAN)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(90.0))
            .build()

        preEndGameParkObservationZone = follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(13.000, 9.000, Point.CARTESIAN),
                    Point(32.000, 15.000, Point.CARTESIAN)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(360.0), Math.toRadians(360.0))
            .build()

        endGameParkObservationZone = follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(32.000, 15.000, Point.CARTESIAN),
                    Point(12.000, 15.000, Point.CARTESIAN)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(360.0), Math.toRadians(360.0))
            .setReversed(true)
            .build()
    }

    override fun runOpMode() {
        /**
         * This initializes the Follower and creates the PathChain. Additionally, this
         * initializes the FTC Dashboard telemetry.
         */
        robot.initialize()

        pathTimer = Timer()
        Constants.setConstants(FConstants::class.java, LConstants::class.java)
        follower.setStartingPose(startPose)
        buildPaths()

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready")
        telemetry.update()

        waitForStart()

        if (isStopRequested) return

        if (isStarted) {
            robot.moveIntakeSlideIn()
            robot.moveArm(ArmState.Transfer)
            robot.moveBucket(BucketState.UP)
            telemetry.addLine("Robot Started")
            telemetry.update()
        }

        /**
         * This runs the OpMode, updating the Follower as well as printing out the debug statements to
         * the Telemetry, as well as the FTC Dashboard.
         */
        while (opModeIsActive()) {
            follower.update()
            autonomousPathUpdate()

            // UPDATE ROBOT STATE
            robot.update()

            // PERFORM AUTOMATIONS
            robot.performAutomations()

            // Use this to update the FtcDashboard field diagram with Pedro
            telemetry.addData("Path State", pathState)
            telemetry.addData("Position", follower.pose.toString())
            telemetry.addData("Path Timer Seconds", pathTimer.elapsedTimeSeconds)

            telemetry.update()

            // Use this to update the FtcDashboard field diagram with Pedro
            follower.telemetryDebug(telemetry)
        }
    }

    private fun autonomousPathUpdate() {
        when (pathState) {
            /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                - Check Robot States: (ArmState, BucketState, IntakeSlideState, OuttakeSlideState)
                */

            PathState.START_WITH_PRELOAD -> {
                follower.followPath(prePushSample1)
                pathState = PathState.PRE_PUSH_SAMPLE_1
            }

            PathState.PRE_PUSH_SAMPLE_1 -> if (!follower.isBusy) {
                follower.followPath(pushSample1ToObservationZone)
                pathState = PathState.PUSH_SAMPLE_1_TO_OBSERVATION_ZONE
            }

            PathState.PUSH_SAMPLE_1_TO_OBSERVATION_ZONE -> if (!follower.isBusy) {
                follower.followPath(prePushSample2)
                pathState = PathState.PRE_PUSH_SAMPLE_2
            }

            PathState.PRE_PUSH_SAMPLE_2 -> if (!follower.isBusy) {
                follower.followPath(pushSample2ToObservationZone)
                pathState = PathState.PUSH_SAMPLE_2_TO_OBSERVATION_ZONE
            }

            PathState.PUSH_SAMPLE_2_TO_OBSERVATION_ZONE -> if (!follower.isBusy) {
                follower.followPath(prePushSample3)
                pathState = PathState.PRE_PUSH_SAMPLE_3
            }

            PathState.PRE_PUSH_SAMPLE_3 -> if (!follower.isBusy) {
                follower.followPath(pushSample3ToObservationZone)
                pathState = PathState.PUSH_SAMPLE_3_TO_OBSERVATION_ZONE
            }

            PathState.PUSH_SAMPLE_3_TO_OBSERVATION_ZONE -> if (!follower.isBusy) {
                follower.followPath(preEndGameParkObservationZone)
                pathState = PathState.PRE_END_GAME_PARK_OBSERVATION_ZONE
            }

            PathState.PRE_END_GAME_PARK_OBSERVATION_ZONE -> if (!follower.isBusy) {
                follower.followPath(endGameParkObservationZone, true)
                pathState = PathState.END_GAME_PARK_OBSERVATION_ZONE
            }

            PathState.END_GAME_PARK_OBSERVATION_ZONE -> Unit
        }
    }
}
