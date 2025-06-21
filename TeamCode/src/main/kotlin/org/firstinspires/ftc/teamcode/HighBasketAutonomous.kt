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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robot.ArmState
import org.firstinspires.ftc.teamcode.robot.BucketState
import org.firstinspires.ftc.teamcode.robot.OuttakeSlideState
import org.firstinspires.ftc.teamcode.robot.Robot
import pedroPathing.constants.FConstants
import pedroPathing.constants.LConstants

@Autonomous(name = "High Basket Auto", group = "Robot")
class HighBasketAutonomous : LinearOpMode() {

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
        PRE_SCORE_PRELOAD,
        SCORE_PRELOAD,
        PICKUP_SAMPLE_1,
        PRE_SCORE_SAMPLE_1,
        SCORE_SAMPLE_1,
        PICKUP_SAMPLE_2,
        PRE_SCORE_SAMPLE_2,
        SCORE_SAMPLE_2,
        PICKUP_SAMPLE_3,
        SCORE_SAMPLE_3,
        PRE_SCORE_SAMPLE_3,
        PRE_END_GAME_LEVEL_ONE_ASCENT,
        END_GAME_LEVEL_ONE_ASCENT
    }

    private var pathState = PathState.START_WITH_PRELOAD
        set(value) {
            field = value
            pathTimer.resetTimer()
        }

    private val startPose = Pose(9.000, 111.000, Math.toRadians(270.0)) // Starting position
    private val preScorePose = Pose(18.000, 128.000, Math.toRadians(315.0)) // Pre Scoring position
    private val scorePose = Pose(15.000, 132.000, Math.toRadians(315.0)) // Scoring position

    private val sample1Pose = Pose(21.50, 133.50, Math.toRadians(0.0)) // First sample Sample
    private val sample2Pose = Pose(25.500, 124.500, Math.toRadians(0.0)) // Second sample Sample
    private val sample3Pose = Pose(25.000, 133.000, Math.toRadians(0.0)) // Third sample Sample

    private var preScorePreload: Path? = null
    private var scorePreload: PathChain? = null
    private var pickupSample1: PathChain? = null
    private var pickupSample2: PathChain? = null
    private var pickupSample3: PathChain? = null
    private var preScoreSample1: PathChain? = null
    private var preScoreSample2: PathChain? = null
    private var preScoreSample3: PathChain? = null
    private var prePark: PathChain? = null
    private var park: PathChain? = null

    fun buildPaths() {
        // Path for scoring preload
        preScorePreload = Path(BezierLine(Point(startPose), Point(preScorePose))).apply {
            setLinearHeadingInterpolation(startPose.heading, preScorePose.heading)
        }

        scorePreload = follower.pathBuilder()
            .addPath(BezierLine(Point(preScorePose), Point(scorePose)))
            .setLinearHeadingInterpolation(preScorePose.heading, scorePose.heading)
            .build()

        pickupSample1 = follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(scorePose),
                    Point(sample3Pose)
                )
            )
            .addParametricCallback(.50) { robot.moveOuttakeSlide(OuttakeSlideState.Collapsed) }
            .addParametricCallback(.8) { robot.moveIntakeSlideOut() }
            .setLinearHeadingInterpolation(scorePose.heading, sample1Pose.heading)
            .build()

        preScoreSample1 = follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(sample1Pose),
                    Point(preScorePose)
                )
            )
            .addParametricCallback(.25) { robot.moveOuttakeSlide(OuttakeSlideState.ScoringInHighBasket) }
            .setLinearHeadingInterpolation(sample1Pose.heading, preScorePose.heading)
            .build()

        pickupSample2 = follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(scorePose),
                    Point(sample2Pose)
                )
            )
            .addParametricCallback(.50) { robot.moveOuttakeSlide(OuttakeSlideState.Collapsed) }
            .addParametricCallback(.85) { robot.moveIntakeSlideOut() }
            .addParametricCallback(.99) { robot.moveIntakeSlideIn() }
            .setLinearHeadingInterpolation(scorePose.heading, sample2Pose.heading)
            .build()

        preScoreSample2 = follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(sample2Pose),
                    Point(preScorePose)
                )
            )
            .addParametricCallback(.25) { robot.moveOuttakeSlide(OuttakeSlideState.ScoringInHighBasket) }
            .setLinearHeadingInterpolation(sample2Pose.heading, preScorePose.heading)
            .build()

        pickupSample3 = follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(scorePose),
                    Point(sample3Pose)
                )
            )
            .addParametricCallback(.50) { robot.moveOuttakeSlide(OuttakeSlideState.Collapsed) }
            .setLinearHeadingInterpolation(scorePose.heading, sample3Pose.heading)
            .build()

        preScoreSample3 = follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(sample3Pose),
                    Point(preScorePose)
                )
            )
            .setLinearHeadingInterpolation(sample3Pose.heading, preScorePose.heading)
            .build()

        // Curved path for parking
        prePark = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(15.000, 132.000, Point.CARTESIAN),
                    Point(24.000, 117.000, Point.CARTESIAN),
                    Point(26.000, 122.000, Point.CARTESIAN),
                    Point(61.000, 98.000, Point.CARTESIAN),
                    Point(60.000, 117.000, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addParametricCallback(.40) { robot.moveOuttakeSlide(OuttakeSlideState.Collapsed) }
            .build()

        park = follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(60.000, 117.000, Point.CARTESIAN),
                    Point(60.000, 98.0, Point.CARTESIAN)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(90.0))
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
                /* Start Pose to Score Preload */
                robot.moveOuttakeSlide(OuttakeSlideState.ScoringInHighBasket)

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
                if (robot.isBucketUp()) robot.moveBucket(BucketState.DOWN)
                /* Score Preload to Pickup Sample 1 */

                if (pathTimer.elapsedTimeSeconds > 1) {
                    robot.moveArm(ArmState.IntakePickup)
                }
                /* Since this is a pathChain, we can have Pedro hold the end point while we are picking up the sample */
                if (pathTimer.elapsedTimeSeconds > 2.75) {
                    follower.followPath(pickupSample1, true)
                    pathState = PathState.PICKUP_SAMPLE_1
                    if (robot.isBucketDown()) robot.moveBucket(BucketState.UP)
                }
            }

            PathState.PICKUP_SAMPLE_1 -> if (!follower.isBusy) {
                robot.moveIntakeSlideIn()
                if (robot.isArmInTransfer() && pathTimer.elapsedTimeSeconds > 2.0) {
                    robot.spinIntakeOut()
                    if (pathTimer.elapsedTimeSeconds > 3.0) {
                        follower.followPath(preScoreSample1, true)
                        pathState = PathState.PRE_SCORE_SAMPLE_1
                    }
                } else if (robot.isOuttakeSlideCollapsed()) {
                    robot.moveArm(ArmState.Transfer)
                }
            }

            PathState.PRE_SCORE_SAMPLE_1 -> if (!follower.isBusy && robot.isOuttakeSlideInScoringInHighBasket()) {
                robot.stopSpinningIntake()
                follower.followPath(scorePreload, true)
                pathState = PathState.SCORE_SAMPLE_1
            }

            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
            PathState.SCORE_SAMPLE_1 -> if (!follower.isBusy) {
                if (robot.isBucketUp()) robot.moveBucket(BucketState.DOWN)

                if (pathTimer.elapsedTimeSeconds > 1) {
                    robot.moveArm(ArmState.IntakePickup)
                }
                /* Since this is a pathChain, we can have Pedro hold the end point while we are picking up the sample */
                if (pathTimer.elapsedTimeSeconds > 2.75) {
                    follower.followPath(pickupSample2, true)
                    pathState = PathState.PICKUP_SAMPLE_2
                    if (robot.isBucketDown()) robot.moveBucket(BucketState.UP)
                }
            }

            PathState.PICKUP_SAMPLE_2 -> if (!follower.isBusy && robot.isOuttakeSlideCollapsed()) {
                if (robot.isArmInTransfer() && pathTimer.elapsedTimeSeconds > 2.0) {
                    robot.spinIntakeOut()
                    if (pathTimer.elapsedTimeSeconds > 3.0) {
                        follower.followPath(preScoreSample2, true)
                        pathState = PathState.PRE_SCORE_SAMPLE_2
                    }
                } else {
                    robot.moveArm(ArmState.Transfer)
                }
            }

            PathState.PRE_SCORE_SAMPLE_2 -> if (!follower.isBusy && robot.isOuttakeSlideInScoringInHighBasket()) {
                robot.stopSpinningIntake()
                follower.followPath(scorePreload, true)
                pathState = PathState.SCORE_SAMPLE_2
            }

            PathState.SCORE_SAMPLE_2 -> if (!follower.isBusy) {
                if (robot.isBucketUp()) robot.moveBucket(BucketState.DOWN)

                /* Since this is a pathChain, we can have Pedro hold the end point while we are picking up the sample */
                if (pathTimer.elapsedTimeSeconds > 2.25) {
                    follower.followPath(prePark, true)
                    pathState = PathState.PRE_END_GAME_LEVEL_ONE_ASCENT
                    if (robot.isBucketDown()) robot.moveBucket(BucketState.UP)
                }
            }

//
//            PathState.PICKUP_SAMPLE_3 -> if (!follower.isBusy) {
//                if (robot.isArmInTransfer()) {
//                    robot.performAutomations() // TODO: Spin out picked up sample???
//                    robot.moveOuttakeSlide(OuttakeSlideState.ScoringInHighBasket)
//                    follower.followPath(preScoreSample3, true)
//                    pathState = PathState.PRE_SCORE_SAMPLE_3
//                } else {
//                    robot.moveArm(ArmState.Transfer)
//                }
//            }
//
//            PathState.PRE_SCORE_SAMPLE_3 -> if (!follower.isBusy && robot.isOuttakeSlideInScoringInHighBasket()) {
//                follower.followPath(scorePreload, true)
//                pathState = PathState.SCORE_SAMPLE_3
//            }
//
//            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//            PathState.SCORE_SAMPLE_3 -> if (!follower.isBusy) {
//                robot.moveBucket(BucketState.DOWN)
//
//                /* Since this is a pathChain, we can have Pedro hold the end point while we are picking up the sample */
//                if (pathTimer.elapsedTimeSeconds > 2.5) {
//                    follower.followPath(prePark, true)
//                    pathState = PathState.PRE_END_GAME_LEVEL_ONE_ASCENT
//                    robot.moveBucket(BucketState.UP)
//                }
//            }
            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
            PathState.PRE_END_GAME_LEVEL_ONE_ASCENT -> if (!follower.isBusy && robot.isOuttakeSlideCollapsed()) {
                /* Score Sample 3 to End Game  */

                /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                follower.followPath(park, true)
                robot.moveOuttakeSlide(OuttakeSlideState.LevelOneAscent)
                pathState = PathState.END_GAME_LEVEL_ONE_ASCENT
            }

            PathState.END_GAME_LEVEL_ONE_ASCENT -> Unit

            else -> Unit
        }
    }
}