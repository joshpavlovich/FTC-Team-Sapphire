package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.extension.initializeForRunToPosition
import org.firstinspires.ftc.teamcode.extension.runToPosition
import kotlin.time.ExperimentalTime

private const val DRIVE_SPEED: Double = 0.4

private const val SECONDS_TO_NET_ZONE_FROM_START: Double = 1.65
private const val SECONDS_TO_OBSERVATION_ZONE_FROM_NET_ZONE: Double = 3.00

private const val HARDWARE_MAP_FRONT_LEFT_MOTOR = "frontLeftMotor"
private const val HARDWARE_MAP_FRONT_RIGHT_MOTOR = "frontRightMotor"
private const val HARDWARE_MAP_BACK_LEFT_MOTOR = "backLeftMotor"
private const val HARDWARE_MAP_BACK_RIGHT_MOTOR = "backRightMotor"
private const val HARDWARE_MAP_BUCKET_SERVO_MOTOR = "bucketServo"
private const val HARDWARE_MAP_SLIDE_MOTOR = "slideMotor"

// Encoder Resolution for Viper Slide 223 RPM Motor = ((((1+(46/11))) * (1+(46/11))) * 28)
// From https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-26-9-1-ratio-24mm-length-8mm-rex-shaft-223-rpm-3-3-5v-encoder/
// Ticks Per Revolution = (Motor Encoder Resolution / Diameter Millimeters)
private const val SLIDE_LIFT_TICKS_PER_MM =
    (751.8) / 120 // Encoder Resolution Formula ->	((((1+(46/11))) * (1+(46/11))) * 28) = 751.8

// Distance in Millimeters for High Basket scoring position = high basket height in Millimeters * Viper Slide Lift Ticks Per Millimeter
private const val SLIDE_LIFT_COLLAPSED = 0.0 * SLIDE_LIFT_TICKS_PER_MM
private const val SLIDE_LIFT_SCORING_IN_HIGH_BASKET = 976.0 * SLIDE_LIFT_TICKS_PER_MM

private const val TELEMETRY_KEY_ROTATIONS = "Rotations"
private const val TELEMETRY_KEY_SPEED = "Speed"

private const val BUCKET_SERVO_INIT_POSITION = 0.0
private const val BUCKET_SERVO_START_POSITION = 0.20
private const val BUCKET_SERVO_END_POSITION = 0.50

@Autonomous(name = "HIGH BASKET Autonomous", group = "Robot")
class HighBasketMecanumAutonomous : LinearOpMode() {

    // DECLARE OUR MOTORS
    // MAKE SURE YOUR ID'S MATCH YOUR CONFIGURATION
    private val frontLeftMotor: DcMotor by lazy {
        hardwareMap.dcMotor.get(HARDWARE_MAP_FRONT_LEFT_MOTOR)
    }
    private val backLeftMotor: DcMotor by lazy {
        hardwareMap.dcMotor.get(HARDWARE_MAP_BACK_LEFT_MOTOR)
    }
    private val frontRightMotor: DcMotor by lazy {
        hardwareMap.dcMotor.get(HARDWARE_MAP_FRONT_RIGHT_MOTOR)
    }
    private val backRightMotor: DcMotor by lazy {
        hardwareMap.dcMotor.get(HARDWARE_MAP_BACK_RIGHT_MOTOR)
    }

    private val bucketServo: Servo by lazy {
        hardwareMap.servo.get(HARDWARE_MAP_BUCKET_SERVO_MOTOR)
    }

    private val slideMotor: DcMotorEx by lazy {
        hardwareMap.dcMotor.get(HARDWARE_MAP_SLIDE_MOTOR) as DcMotorEx
    }

    enum class AutoState {
        START, TO_NET_ZONE, SLIDE_UP, HIGH_BASKET_SCORE, SLIDE_DOWN, TO_OBSERVATION_ZONE, END
    }

    private var state = AutoState.START

    @OptIn(ExperimentalTime::class)
    override fun runOpMode() {
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        frontRightMotor.direction = Direction.FORWARD
        backRightMotor.direction = Direction.FORWARD

        // Setting zeroPowerBehavior to BRAKE enables a "brake mode".
        // This causes the motor to slow down much faster when it is coasting.
        // This creates a much more controllable drivetrain. As the robot
        // stops much quicker.
        frontLeftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        frontRightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backLeftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backRightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        bucketServo.direction = Servo.Direction.REVERSE
        bucketServo.position = BUCKET_SERVO_INIT_POSITION

        slideMotor.initializeForRunToPosition(SLIDE_LIFT_COLLAPSED, Direction.REVERSE, true)

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready")
        telemetry.update()

        val autoTimer = ElapsedTime()

        waitForStart()

        if (isStopRequested) return

        if (isStarted) {
            bucketServo.position = BUCKET_SERVO_START_POSITION
            telemetry.addLine("Robot Started")
            telemetry.update()
            autoTimer.reset()
        }

        while (opModeIsActive()) {

            telemetry.addData("Auto State", state.name)
            telemetry.update()

            when (state) {
                AutoState.START -> {
                    backward(SECONDS_TO_NET_ZONE_FROM_START)
                    state = AutoState.TO_NET_ZONE
                }

                AutoState.HIGH_BASKET_SCORE -> {
                    bucketServo.position = BUCKET_SERVO_END_POSITION
                    if (autoTimer.seconds() > 7) {
                        state = AutoState.SLIDE_DOWN
                    }
                }

                AutoState.SLIDE_UP -> {
                    slideMotor.runToPosition(SLIDE_LIFT_SCORING_IN_HIGH_BASKET, 2100.0)

                    if (autoTimer.seconds() > 5 && slideMotor.currentPosition >= SLIDE_LIFT_SCORING_IN_HIGH_BASKET.toInt()) {
                        state = AutoState.HIGH_BASKET_SCORE
                    }
                }

                AutoState.SLIDE_DOWN -> {
                    bucketServo.position = BUCKET_SERVO_START_POSITION
                    if (autoTimer.seconds() > 25) {
                        state = AutoState.TO_OBSERVATION_ZONE
                    }
                }

                AutoState.TO_NET_ZONE -> {
                    if (autoTimer.seconds() > 1.25) {
                        state = AutoState.SLIDE_UP
                    }
                }

                AutoState.TO_OBSERVATION_ZONE -> {
                    slideMotor.runToPosition(SLIDE_LIFT_COLLAPSED, 2100.0)

                    if (!slideMotor.isBusy && slideMotor.targetPosition <= 0) {
                        telemetry.addData("Slide not busy", slideMotor.currentPosition)
                        slideMotor.initializeForRunToPosition(
                            SLIDE_LIFT_COLLAPSED,
                            Direction.REVERSE,
                            true
                        )
                    }

                    forward(SECONDS_TO_OBSERVATION_ZONE_FROM_NET_ZONE) { state = AutoState.END }
                }

                AutoState.END -> Unit
            }

            // ADD TELEMETRY DATA AND UPDATE
            telemetry.addData(TELEMETRY_KEY_ROTATIONS, frontLeftMotor.currentPosition)
            telemetry.addData(TELEMETRY_KEY_SPEED, frontLeftMotor.power)
            telemetry.addData("Auto State", state.name)
            telemetry.addData("Auto Timer", autoTimer.seconds())
            telemetry.update()
        }
    }

    fun backward(time: Double, onComplete: () -> Unit = {}) {
        val timer = ElapsedTime()

        waitForStart()

        while (timer.seconds() <= time) {
            move(-1.0, 1.0, -1.0, -1.0)
        }

        if (timer.seconds() > time) {
            onComplete()
        }

        move(0.0, 0.0, 0.0, 0.0)
    }

    fun buffer(time: Double, onComplete: () -> Unit = {}) {
        val timer = ElapsedTime()

        waitForStart()

        while (timer.seconds() <= time) {
            move(0.0, 0.0, 0.0, 0.0)
        }
    }

    fun forward(time: Double, onComplete: () -> Unit = {}) {
        val timer = ElapsedTime()

        waitForStart()

        while (timer.seconds() <= time) {
            move(1.0, -1.0, 1.0, 1.0)
        }

        if (timer.seconds() > time) {
            onComplete()
        }

        move(0.0, 0.0, 0.0, 0.0)
    }

    fun strafe(strafeLeft: Boolean, time: Double, onComplete: () -> Unit = {}) {
        val timer = ElapsedTime()

        waitForStart()

        while (timer.seconds() <= time) {
            if (strafeLeft) {
                move(-1.0, 1.0, -1.0, 1.0)
            } else {
                move(1.0, -1.0, 1.0, -1.0)
            }
        }

        move(0.0, 0.0, 0.0, 0.0)
    }

    fun move(leftFront: Double, leftBack: Double, rightFront: Double, rightBack: Double) {
        val driveSpeed = getDriveSpeed()
        frontLeftMotor.power = leftFront * driveSpeed
        backLeftMotor.power = leftBack * driveSpeed
        frontRightMotor.power = rightFront * driveSpeed
        backRightMotor.power = rightBack * driveSpeed
    }

    fun getDriveSpeed(): Double = DRIVE_SPEED
}