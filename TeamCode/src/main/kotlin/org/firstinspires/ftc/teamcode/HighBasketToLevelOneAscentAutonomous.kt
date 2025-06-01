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

@Autonomous(name = "High Basket To Level One Ascent Auto", group = "Robot")
class HighBasketToLevelOneAscentAutonomous : LinearOpMode() {

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
        PRE_END_GAME_LEVEL_ONE_ASCENT,
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

    private val parkControlPose0 = Pose(83.0, 108.0) // Control point 0 for curved path
    private val parkControlPose1 = Pose(58.0, 98.0) // Control point 1 for curved path
    private val parkControlPose2 = Pose(63.0, 102.0) // Control point 2 for curved path
    private val parkControlPose3 = Pose(83.000, 112.539, Math.toRadians(90.0)) // Control point 3 for curved path
    private val parkPose = Pose(83.0, 96.0, Math.toRadians(90.0)) // Parking position

    private var preScorePreload: Path? = null
    private var scorePreload: Path? = null
    private var prePark: PathChain? = null
    private var park: PathChain? = null

    fun buildPaths() {
        // Path for scoring preload
        preScorePreload = Path(BezierLine(Point(startPose), Point(preScorePose))).apply {
            setLinearHeadingInterpolation(startPose.heading, preScorePose.heading)
        }

        scorePreload = Path(BezierLine(Point(preScorePose), Point(scorePose))).apply {
            setLinearHeadingInterpolation(preScorePose.heading, scorePose.heading)
        }

        // Curved path for parking
        prePark = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(15.000, 132.000, Point.CARTESIAN),
                    Point(50.000, 100.000, Point.CARTESIAN),
                    Point(68.000, 113.000, Point.CARTESIAN),
                    Point(83.000, 86.000, Point.CARTESIAN),
                    Point(83.000, 112.539, Point.CARTESIAN)
                )
            )
            .setTangentHeadingInterpolation()
            .addParametricCallback(.10) { robot.moveOuttakeSlide(OuttakeSlideState.Collapsed) }
            .build()

        park = follower.pathBuilder()
            .addPath(
                BezierLine(
                    Point(83.000, 112.539, Point.CARTESIAN),
                    Point(83.000, 98.0, Point.CARTESIAN)
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
                    follower.followPath(prePark, true)
                    pathState = PathState.PRE_END_GAME_LEVEL_ONE_ASCENT
                    robot.moveBucket(BucketState.UP)
                }
            }

            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
            PathState.PRE_END_GAME_LEVEL_ONE_ASCENT -> if (!follower.isBusy && robot.isOuttakeSlideCollapsed()) {
                /* Score Sample 3 to End Game  */

                /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                follower.followPath(park, true)
                robot.moveOuttakeSlide(OuttakeSlideState.LevelOneAscent)
                pathState = PathState.END_GAME_LEVEL_ONE_ASCENT
            }

            PathState.END_GAME_LEVEL_ONE_ASCENT -> Unit
        }
    }
}
