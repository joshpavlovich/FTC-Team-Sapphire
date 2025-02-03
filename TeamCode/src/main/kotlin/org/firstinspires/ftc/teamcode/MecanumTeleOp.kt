package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import kotlin.math.abs
import kotlin.math.max

private const val DRIVE_SPEED: Double = 0.5

private const val HARDWARE_MAP_FRONT_LEFT_MOTOR = "frontLeftMotor"
private const val HARDWARE_MAP_FRONT_RIGHT_MOTOR = "frontRightMotor"
private const val HARDWARE_MAP_BACK_LEFT_MOTOR = "backLeftMotor"
private const val HARDWARE_MAP_BACK_RIGHT_MOTOR = "backRightMotor"
private const val HARDWARE_MAP_SLIDE_MOTOR = "slideMotor"
private const val HARDWARE_MAP_BUCKET_SERVO_MOTOR = "bucketServo"
private const val HARDWARE_MAP_INTAKE_SLIDE_SERVO_MOTOR = "intakeSlideServo"
private const val HARDWARE_MAP_INTAKE_ARM_MOTOR = "intakeArmMotor"
private const val HARDWARE_MAP_INTAKE_SERVO_MOTOR = "intakeServo"

private const val SLIDE_LIFT_TICKS_PER_MM = (751.8) / 120
private const val ARM_MOTOR_TICKS_PER_MM = (1992.6) / 96.0 // IS THIS THE CORRECT DIAMETER???
private const val SLIDE_LIFT_COLLAPSED = 0.0 * SLIDE_LIFT_TICKS_PER_MM
private const val SLIDE_LIFT_SCORING_IN_LOW_BASKET = 806.52 * SLIDE_LIFT_TICKS_PER_MM
private const val SLIDE_LIFT_SCORING_IN_HIGH_BASKET = 976.0 * SLIDE_LIFT_TICKS_PER_MM

private const val BUCKET_SERVO_INIT_POSITION = 0.0
private const val BUCKET_SERVO_START_POSITION = 0.20
private const val BUCKET_SERVO_END_POSITION = 0.50

private const val INTAKE_SLIDE_SERVO_START_POSITION = 0.0
private const val INTAKE_SLIDE_SERVO_END_POSITION = 0.28

private const val INTAKE_ARM_START_POSITION = 2.35 * ARM_MOTOR_TICKS_PER_MM
private const val INTAKE_ARM_END_POSITION = 48.3 * ARM_MOTOR_TICKS_PER_MM

private const val TELEMETRY_KEY_ROTATIONS = "Rotations"
private const val TELEMETRY_KEY_SPEED = "Speed"
private const val TELEMETRY_KEY_Y_VALUE = "Y Value"

@TeleOp(name = "Team Sapphire: Mecanum TeleOp", group = "Robot")
class MecanumTeleOp : LinearOpMode() {

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

    private val slideMotor: DcMotorEx by lazy {
        hardwareMap.dcMotor.get(HARDWARE_MAP_SLIDE_MOTOR) as DcMotorEx
    }

    private val bucketServo: Servo by lazy {
        hardwareMap.servo.get(HARDWARE_MAP_BUCKET_SERVO_MOTOR)
    }

    private val intakeArmMotor: DcMotorEx by lazy {
        hardwareMap.dcMotor.get(HARDWARE_MAP_INTAKE_ARM_MOTOR) as DcMotorEx
    }

    private val intakeSlideServo: Servo by lazy {
        hardwareMap.servo.get(HARDWARE_MAP_INTAKE_SLIDE_SERVO_MOTOR)
    }

    private val intakeServo: CRServo by lazy {
        hardwareMap.crservo.get(HARDWARE_MAP_INTAKE_SERVO_MOTOR)
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

        slideMotor.initializeForRunToPosition(SLIDE_LIFT_COLLAPSED, Direction.REVERSE, true)

        // Initialize the arm motor to zero
        intakeArmMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intakeArmMotor.direction = Direction.FORWARD
        intakeArmMotor.targetPosition = 0
        intakeArmMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        intakeArmMotor.mode = DcMotor.RunMode.RUN_TO_POSITION

        intakeArmMotor.runToPosition(INTAKE_ARM_START_POSITION, 800.0)

        intakeSlideServo.direction = Servo.Direction.REVERSE
        intakeSlideServo.position = INTAKE_SLIDE_SERVO_START_POSITION

        bucketServo.direction = Servo.Direction.REVERSE
        bucketServo.position = BUCKET_SERVO_INIT_POSITION

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready")
        telemetry.update()

        waitForStart()

        if (isStopRequested) return

        if (isStarted) {
            bucketServo.position = BUCKET_SERVO_START_POSITION
            telemetry.addLine("Robot Started")
            telemetry.update()
        }

        while (opModeIsActive()) {
            // START SETUP MECANUM DRIVETRAIN MOTORS
            // Remember, Y stick value is reversed
            val leftStickY: Double = -gamepad1.left_stick_y.toDouble()
            val leftStickX: Double = gamepad1.left_stick_x.toDouble() * 1.1
            val rightStickX: Double = gamepad1.right_stick_x.toDouble()

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            val denominator: Double = max(abs(leftStickY) + abs(leftStickX) + abs(rightStickX), 1.0)
            val frontLeftPower = (leftStickY + leftStickX + rightStickX) / denominator
            val backLeftPower = (leftStickY - leftStickX + rightStickX) / denominator
            val frontRightPower = (leftStickY - leftStickX - rightStickX) / denominator
            val backRightPower = (leftStickY + leftStickX - rightStickX) / denominator

            // Limit speed to MaxPower
            val maxPower: Double = if (gamepad1.left_trigger == 0F) {
                DRIVE_SPEED
            } else {
                DRIVE_SPEED + ((1 - DRIVE_SPEED) * gamepad1.left_trigger)
            }

            frontLeftMotor.power = frontLeftPower * maxPower
            backLeftMotor.power = -backLeftPower * maxPower
            frontRightMotor.power = frontRightPower * maxPower
            backRightMotor.power = backRightPower * maxPower
            // END SETUP MECANUM DRIVETRAIN MOTORS

            // START SET SLIDE MOTOR MODE AND POWER
            if (gamepad1.dpad_up) {
                slideMotor.runToPosition(SLIDE_LIFT_SCORING_IN_HIGH_BASKET, 2100.0)
            } else if (gamepad1.dpad_down) {
                slideMotor.runToPosition(SLIDE_LIFT_COLLAPSED, 2100.0)
            }

            if (!slideMotor.isBusy && slideMotor.targetPosition <= 0) {
                telemetry.addData("Slide not busy", slideMotor.currentPosition)
                slideMotor.initializeForRunToPosition(SLIDE_LIFT_COLLAPSED, Direction.REVERSE, true)
            }
            // END SET SLIDE MOTOR MODE AND POWER

            telemetry.addData("Slide motor target position", slideMotor.targetPosition)
            telemetry.addData("Slide motor current position", slideMotor.currentPosition)
            telemetry.addData("Slide motor current", slideMotor.getCurrent(CurrentUnit.AMPS))
            // END GET CURRENT SLIDE STATE AND SET SLIDE MOTOR MODE AND POWER

            // START SET ARM MOTOR MODE AND POWER
            if (gamepad1.y) {
                intakeArmMotor.runToPosition(INTAKE_ARM_END_POSITION, 800.0)
            } else if (gamepad1.a) {
                intakeArmMotor.runToPosition(INTAKE_ARM_START_POSITION, 800.0)
            }
            // END SET ARM MOTOR MODE AND POWER

            telemetry.addData("Intake arm motor target position", intakeArmMotor.targetPosition)
            telemetry.addData("Intake arm motor current position", intakeArmMotor.currentPosition)
            telemetry.addData(
                "Intake arm motor current",
                intakeArmMotor.getCurrent(CurrentUnit.AMPS)
            )
            // END GET CURRENT SLIDE STATE AND SET SLIDE MOTOR MODE AND POWER

            // START SET BUCKET SERVO MOTOR POSITION
            if (gamepad1.x) {
                bucketServo.position = BUCKET_SERVO_END_POSITION
            } else if (gamepad1.b) {
                bucketServo.position = BUCKET_SERVO_START_POSITION
            }

            telemetry.addData("Bucket Servo Position", bucketServo.position)
            // END SET BUCKET SERVO MOTOR POSITION

            // START SET INTAKE SLIDE SERVO MOTOR POSITION
            if (gamepad1.dpad_left) {
                intakeSlideServo.position = INTAKE_SLIDE_SERVO_END_POSITION
            } else if (gamepad1.dpad_right) {
                intakeSlideServo.position = INTAKE_SLIDE_SERVO_START_POSITION
            }

            telemetry.addData("Intake Slide Servo Position", intakeSlideServo.position)
            // END SET INTAKE SLIDE SERVO MOTOR POSITION

            // START SET INTAKE SERVO POWER

            if (gamepad1.left_bumper) {
                intakeServo.direction = Direction.REVERSE
            }

            if (gamepad1.right_bumper) {
                intakeServo.direction = Direction.FORWARD
            }

            intakeServo.power = if (gamepad1.left_bumper || gamepad1.right_bumper) {
                1.0
            } else {
                0.0
            }
            // END SET INTAKE SERVO POWER

            // ADD TELEMETRY DATA AND UPDATE
            telemetry.addData(TELEMETRY_KEY_ROTATIONS, frontLeftMotor.currentPosition)
            telemetry.addData(TELEMETRY_KEY_SPEED, frontLeftMotor.power)
            telemetry.addData(TELEMETRY_KEY_Y_VALUE, leftStickY)
            telemetry.update()
        }
    }

    private fun DcMotorEx.initializeForRunToPosition(
        position: Double,
        direction: Direction,
        setPowerToZero: Boolean = false
    ) {
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        this.direction = direction
        targetPosition = position.toInt()
        mode = DcMotor.RunMode.RUN_TO_POSITION
        mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        if (setPowerToZero) {
            power = 0.0
        }
    }

    private fun DcMotorEx.runToPosition(position: Double, velocity: Double) {
        targetPosition = position.toInt() // the position you want to reach
//        targetPositionTolerance = 1 // set accuracy to 1 tick
        this.velocity = velocity
        mode = DcMotor.RunMode.RUN_TO_POSITION
    }
}