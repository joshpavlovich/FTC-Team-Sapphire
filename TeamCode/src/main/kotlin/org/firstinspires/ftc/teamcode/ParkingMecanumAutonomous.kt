package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime

const val DELAY_SECONDS_PARKING_TO_OBSERVATION_ZONE = 27.5

private const val HARDWARE_MAP_FRONT_LEFT_MOTOR = "frontLeftMotor"
private const val HARDWARE_MAP_FRONT_RIGHT_MOTOR = "frontRightMotor"
private const val HARDWARE_MAP_BACK_LEFT_MOTOR = "backLeftMotor"
private const val HARDWARE_MAP_BACK_RIGHT_MOTOR = "backRightMotor"
private const val HARDWARE_MAP_BUCKET_SERVO_MOTOR = "bucketServo"

private const val BUCKET_SERVO_INIT_POSITION = 0.0
private const val BUCKET_SERVO_START_POSITION = 0.20

private const val TELEMETRY_KEY_ROTATIONS = "Rotations"
private const val TELEMETRY_KEY_SPEED = "Speed"

abstract class ParkingMecanumAutonomous : LinearOpMode() {

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

    private var isParked: Boolean = false

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

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready")
        telemetry.update()

        val autoDelayedTimer = ElapsedTime()

        waitForStart()

        if (isStopRequested) return

        if (isStarted) {
            autoDelayedTimer.reset()
            bucketServo.position = BUCKET_SERVO_START_POSITION
            telemetry.addLine("Robot Started")
            telemetry.update()
        }

        while (opModeIsActive()) {
            if (!isParked && autoDelayedTimer.seconds() >= getSecondsToDelayFromStart()) {
                forward(getSecondsToObservationZoneFromStart()) {isParked = true}
            }

            move(0.0, 0.0, 0.0, 0.0)

            // ADD TELEMETRY DATA AND UPDATE
            telemetry.addData(TELEMETRY_KEY_ROTATIONS, frontLeftMotor.currentPosition)
            telemetry.addData(TELEMETRY_KEY_SPEED, frontLeftMotor.power)
            telemetry.addData("Delay Timer seconds", autoDelayedTimer.seconds())
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


    abstract fun getDriveSpeed(): Double

    abstract fun getSecondsToDelayFromStart(): Double

    abstract fun getSecondsToObservationZoneFromStart(): Double
}