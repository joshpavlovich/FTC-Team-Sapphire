package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime

private const val DRIVE_SPEED: Double = 0.5

private const val HARDWARE_MAP_FRONT_LEFT_MOTOR = "frontLeftMotor"

private const val HARDWARE_MAP_FRONT_RIGHT_MOTOR = "frontRightMotor"
private const val HARDWARE_MAP_BACK_LEFT_MOTOR = "backLeftMotor"
private const val HARDWARE_MAP_BACK_RIGHT_MOTOR = "backRightMotor"
private const val HARDWARE_MAP_BUCKET_SERVO_MOTOR = "bucketServo"

private const val BUCKET_SERVO_INIT_POSITION = 0.0
private const val BUCKET_SERVO_START_POSITION = 0.20

private const val SECONDS_TO_OBSERVATION_ZONE_FROM_START: Int = 2

private const val TELEMETRY_KEY_ROTATIONS = "Rotations"
private const val TELEMETRY_KEY_SPEED = "Speed"

@Autonomous(name = "Team Sapphire: Mecanum Autonomous", group = "Robot")
class MecanumAutonomous : LinearOpMode() {

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

        val timer = ElapsedTime()

        waitForStart()

        if (isStopRequested) return

        if (isStarted) {
            bucketServo.position = BUCKET_SERVO_START_POSITION
            telemetry.addLine("Robot Started")
            telemetry.update()
            timer.reset()
        }

        while (opModeIsActive()) {
            // START SETUP MECANUM DRIVETRAIN MOTORS

            // Limit speed to MaxPower
            val maxPower = DRIVE_SPEED

            while (timer.seconds() <= SECONDS_TO_OBSERVATION_ZONE_FROM_START) {
                frontLeftMotor.power = maxPower
                backLeftMotor.power = -maxPower
                frontRightMotor.power = maxPower
                backRightMotor.power = maxPower
            }

            frontLeftMotor.power = 0.0
            backLeftMotor.power = 0.0
            frontRightMotor.power = 0.0
            backRightMotor.power = 0.0
            // END SETUP MECANUM DRIVETRAIN MOTORS

            // ADD TELEMETRY DATA AND UPDATE
            telemetry.addData(TELEMETRY_KEY_ROTATIONS, frontLeftMotor.currentPosition)
            telemetry.addData(TELEMETRY_KEY_SPEED, frontLeftMotor.power)
            telemetry.update()
        }
    }
}