package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.Path
import com.pedropathing.pathgen.PathChain
import com.pedropathing.pathgen.Point
import com.pedropathing.util.Constants
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robot.ArmState
import org.firstinspires.ftc.teamcode.robot.BucketState
import org.firstinspires.ftc.teamcode.robot.OuttakeSlideState
import org.firstinspires.ftc.teamcode.robot.Robot
import pedroPathing.constants.FConstants
import pedroPathing.constants.LConstants

@Disabled
@Autonomous(name = "High Basket Auto", group = "Robot")
class HighBasketAutonomous : LinearOpMode() {

    // Instance of the "Robot" class
    private val robot = Robot(this)

    private val follower: Follower by lazy {
        Constants.setConstants(FConstants::class.java, LConstants::class.java)
        Follower(hardwareMap, FConstants::class.java, LConstants::class.java)
    }

    private lateinit var pathTimer: Timer
    private lateinit var actionTimer: Timer
    private lateinit var opmodeTimer: Timer

    /**
     * This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method.
     */
    enum class PathState {
        START_WITH_PRELOAD,
        PRE_SCORE_PRELOAD,
        SCORE_PRELOAD,
        PRE_PICKUP_SAMPLE_1,
        PICKUP_SAMPLE_1,
        SCORE_SAMPLE_1,
        PICKUP_SAMPLE_2,
        SCORE_SAMPLE_2,
        PICKUP_SAMPLE_3,
        SCORE_SAMPLE_3,
        END_GAME_LEVEL_ONE_ASCENT
    }

    private var pathState = PathState.START_WITH_PRELOAD
        set(value) {
            field = value
            pathTimer.resetTimer()
        }

    private val startPose = Pose(9.0, 111.0, Math.toRadians(270.0)) // Starting position
    private val preScorePose = Pose(18.0, 128.0, Math.toRadians(315.0)) // Pre Scoring position
    private val scorePose = Pose(15.0, 132.0, Math.toRadians(315.0)) // Scoring position

    private val preSample1Pose = Pose(15.0, 132.0, Math.toRadians(0.0)) // Pre First sample Sample
    private val sample1Pose = Pose(18.0, 132.0, Math.toRadians(0.0)) // First sample Sample
    private val sample2Pose = Pose(43.0, 130.0, Math.toRadians(0.0)) // Second sample Sample
    private val sample3Pose = Pose(49.0, 135.0, Math.toRadians(0.0)) // Third sample Sample

    private val parkPose = Pose(60.0, 98.0, Math.toRadians(90.0)) // Parking position
    private val parkControlPose =
        Pose(85.0, 98.0, Math.toRadians(90.0)) // Control point for curved path

    private var preScorePreload: Path? = null
    private var scorePreload: Path? = null
    private var park: Path? = null
    private var prePickupSample1: PathChain? = null
    private var pickupSample1: PathChain? = null
    private var pickupSample2: PathChain? = null
    private var pickupSample3: PathChain? = null
    private var scoreSample1: PathChain? = null
    private var scoreSample2: PathChain? = null
    private var scoreSample3: PathChain? = null

    fun buildPaths() {
        // Path for scoring preload
        preScorePreload = Path(BezierLine(Point(startPose), Point(preScorePose))).apply {
            setLinearHeadingInterpolation(startPose.heading, preScorePose.heading)
        }

        scorePreload = Path(BezierLine(Point(preScorePose), Point(scorePose))).apply {
            setLinearHeadingInterpolation(preScorePose.heading, scorePose.heading)
        }

        prePickupSample1 = follower.pathBuilder()
            .addPath(BezierLine(Point(scorePose), Point(parkControlPose)))
            .addParametricCallback(.10) { robot.moveOuttakeSlide(OuttakeSlideState.Collapsed) }
            .addParametricCallback(.70) { robot.moveOuttakeSlide(OuttakeSlideState.LevelOneAscent) }
            .setLinearHeadingInterpolation(scorePose.heading, parkControlPose.heading)
            .build()

        // Path chains for picking up and scoring samples
        pickupSample1 = follower.pathBuilder()
            .addPath(BezierLine(Point(preSample1Pose), Point(sample1Pose)))
            .setLinearHeadingInterpolation(preSample1Pose.heading, sample1Pose.heading)
            .build()

        scoreSample1 = follower.pathBuilder()
            .addPath(BezierLine(Point(sample1Pose), Point(scorePose)))
//            .addParametricCallback(.25) { robot.moveOuttakeSlide(OuttakeSlideState.Collapsed) }
            .setLinearHeadingInterpolation(sample1Pose.heading, scorePose.heading)
            .build()

        pickupSample2 = follower.pathBuilder()
            .addPath(BezierLine(Point(scorePose), Point(sample2Pose)))
            .setLinearHeadingInterpolation(scorePose.heading, sample2Pose.heading)
            .build()

        scoreSample2 = follower.pathBuilder()
            .addPath(BezierLine(Point(sample2Pose), Point(scorePose)))
            .setLinearHeadingInterpolation(sample2Pose.heading, scorePose.heading)
            .build()

        pickupSample3 = follower.pathBuilder()
            .addPath(BezierLine(Point(scorePose), Point(sample3Pose)))
            .setLinearHeadingInterpolation(scorePose.heading, sample3Pose.heading)
            .build()

        scoreSample3 = follower.pathBuilder()
            .addPath(BezierLine(Point(sample3Pose), Point(scorePose)))
            .setLinearHeadingInterpolation(sample3Pose.heading, scorePose.heading)
            .build()

        // Curved path for parking
        park = Path(BezierCurve(Point(scorePose), Point(parkControlPose), Point(parkPose))).apply {
            setLinearHeadingInterpolation(scorePose.heading, parkPose.heading)
        }
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

            // ADD AUTO SPIN INTAKE WHEN ARM IS PICKUP STATE

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
                robot.moveOuttakeSlide(OuttakeSlideState.ScoringInHighBasket)
                /* Start Pose to Score Preload */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                follower.followPath(preScorePreload)
                pathState = PathState.PRE_SCORE_PRELOAD
            }

            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
            PathState.PRE_SCORE_PRELOAD -> if (!follower.isBusy && robot.isOuttakeSlideInScoringInHighBasket()) {
                /* Score Preload to Pickup Sample 1 */

                /* Since this is a pathChain, we can have Pedro hold the end point while we are picking up the sample */
                follower.followPath(scorePreload, true)
                pathState = PathState.SCORE_PRELOAD
            }
            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
            PathState.SCORE_PRELOAD -> if (!follower.isBusy) {
                robot.moveBucket(BucketState.DOWN)
                /* Score Preload to Pickup Sample 1 */

                /* Since this is a pathChain, we can have Pedro hold the end point while we are picking up the sample */
                if (pathTimer.elapsedTimeSeconds > 2.5) {
                    follower.followPath(prePickupSample1, true)
                    pathState = PathState.PRE_PICKUP_SAMPLE_1
                    robot.moveBucket(BucketState.UP)
//                    robot.moveArm(ArmState.IntakePickup)
                }
            }

//            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
            PathState.PRE_PICKUP_SAMPLE_1 -> if (!follower.isBusy) {
                robot.moveOuttakeSlide(OuttakeSlideState.LevelOneAscent)
//                /* Pick Sample 1 to Score Sample 1 */

//                robot.moveIntakeSlideOut()
//                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                follower.followPath(pickupSample1, true)
//                pathState = PathState.PICKUP_SAMPLE_1
            }

//            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
            PathState.PICKUP_SAMPLE_1 -> if (!follower.isBusy) {
//                if (pathTimer.elapsedTimeSeconds >= 2.5) {
//                    robot.moveOuttakeSlide(OuttakeSlideState.Collapsed)
//                }
//                /* Pick Sample 1 to Score Sample 1 */
//
//                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                follower.followPath(scoreSample1, true)
//                pathState = PathState.SCORE_SAMPLE_1
            }

//            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//            PathState.SCORE_SAMPLE_1 -> if (!follower.isBusy) {
//                /* Score Sample 1 to Pickup Sample 2 */
//
//                /* Since this is a pathChain, we can have Pedro hold the end point while we are picking up the sample */
//                follower.followPath(pickupSample2, true)
//                pathState = PathState.PICKUP_SAMPLE_2
//            }
//
//            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
//            PathState.PICKUP_SAMPLE_2 -> if (!follower.isBusy) {
//                /* Pick Sample 1 to Score Sample 2 */
//
//                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                follower.followPath(scoreSample2, true)
//                pathState = PathState.SCORE_SAMPLE_2
//            }
//
//            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//            PathState.SCORE_SAMPLE_2 -> if (!follower.isBusy) {
//                /* Score Sample 2 to Pickup Sample 3 */
//
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                follower.followPath(pickupSample3, true)
//                pathState = PathState.PICKUP_SAMPLE_3
//            }
//
//            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
//            PathState.PICKUP_SAMPLE_3 -> if (!follower.isBusy) {
//                /* Pick Sample 3 to Score Sample 2 */
//
//                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                follower.followPath(scoreSample2, true)
//                pathState = PathState.SCORE_SAMPLE_3
//            }
//
//            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//            PathState.SCORE_SAMPLE_3 -> if (!follower.isBusy) {
//                /* Score Sample 3 to End Game  */
//
//                /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
//                follower.followPath(park, true)
//                pathState = PathState.END_GAME_LEVEL_ONE_ASCENT
//            }
//
//            PathState.END_GAME_LEVEL_ONE_ASCENT -> if (!follower.isBusy) {
//                /* Level 1 Ascent */
//.               robot.moveOuttakeSlide(OuttakeSlideState.LevelOneAscent)
//                /* Set the state to a Case we won't use or define, so it just stops running an new paths */
//                follower.pose = parkPose // IS THIS CORRECT???
//            }
            else -> {}
        }
    }
}
