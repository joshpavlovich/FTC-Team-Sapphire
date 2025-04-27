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
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.robot.Robot
import pedroPathing.constants.FConstants
import pedroPathing.constants.LConstants

@Autonomous(name = "Pedro Auto", group = "Robot")
class PedroAuto : OpMode() {

    // Instance of the "Robot" class
    private val robot = Robot(this)

    private var follower: Follower? = null

    private var pathTimer: Timer? = null
    private val actionTimer: Timer? = null
    private val opmodeTimer: Timer? = null

    /**
     * This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method.
     */
    private var pathState = 0

    private val startPose = Pose(9.0, 111.0, Math.toRadians(270.0)) // Starting position
    private val scorePose = Pose(14.0, 129.0, Math.toRadians(315.0)) // Scoring position

    private val pickup1Pose = Pose(37.0, 121.0, Math.toRadians(0.0)) // First sample pickup
    private val pickup2Pose = Pose(43.0, 130.0, Math.toRadians(0.0)) // Second sample pickup
    private val pickup3Pose = Pose(49.0, 135.0, Math.toRadians(0.0)) // Third sample pickup

    private val parkPose = Pose(60.0, 98.0, Math.toRadians(90.0)) // Parking position
    private val parkControlPose =
        Pose(60.0, 98.0, Math.toRadians(90.0)) // Control point for curved path

    private var scorePreload: Path? = null
    private var park: Path? = null
    private var grabPickup1: PathChain? = null
    private var grabPickup2: PathChain? = null
    private var grabPickup3: PathChain? = null
    private var scorePickup1: PathChain? = null
    private var scorePickup2: PathChain? = null
    private var scorePickup3: PathChain? = null

    fun buildPaths() {
        // Path for scoring preload
        scorePreload = Path(BezierLine(Point(startPose), Point(scorePose)))
        scorePreload!!.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())

        // Path chains for picking up and scoring samples
        grabPickup1 = follower!!.pathBuilder()
            .addPath(BezierLine(Point(scorePose), Point(pickup1Pose)))
            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
            .build()

        scorePickup1 = follower!!.pathBuilder()
            .addPath(BezierLine(Point(pickup1Pose), Point(scorePose)))
            .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
            .build()

        grabPickup2 = follower!!.pathBuilder()
            .addPath(BezierLine(Point(scorePose), Point(pickup2Pose)))
            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
            .build()

        scorePickup2 = follower!!.pathBuilder()
            .addPath(BezierLine(Point(pickup2Pose), Point(scorePose)))
            .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
            .build()

        grabPickup3 = follower!!.pathBuilder()
            .addPath(BezierLine(Point(scorePose), Point(pickup3Pose)))
            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
            .build()

        scorePickup3 = follower!!.pathBuilder()
            .addPath(BezierLine(Point(pickup3Pose), Point(scorePose)))
            .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
            .build()

        // Curved path for parking
        park = Path(BezierCurve(Point(scorePose), Point(parkControlPose), Point(parkPose)))
        park!!.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
    }

    private val telemetryA: Telemetry? = null

    /**
     * This initializes the Follower and creates the PathChain. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    override fun init() {
        pathTimer = Timer()
        Constants.setConstants(FConstants::class.java, LConstants::class.java)
        follower = Follower(hardwareMap)
        follower!!.setStartingPose(startPose)
        buildPaths()
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    override fun loop() {
        follower!!.update()
        autonomousPathUpdate()

        // Use this to update the FtcDashboard field diagram with Pedro
        follower!!.telemetryDebug(telemetry)

        // Use this to update the FtcDashboard field diagram with Pedro
        telemetry.addData("Path State", pathState)
        telemetry.addData("Position", follower!!.getPose().toString())
        telemetry.update()
    }


    fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                follower!!.followPath(scorePreload)
                setPathState(1)
            }

            1 -> if (!follower!!.isBusy()) {
                follower!!.followPath(grabPickup1, true)
                setPathState(2)
            }
        }
    }

    fun setPathState(pState: Int) {
        pathState = pState
        pathTimer!!.resetTimer()
    }
}
