package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
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
private const val HARDWARE_MAP_INTAKE_SLIDE_SERVO_MOTOR = "intakeSlideServo"
private const val HARDWARE_MAP_INTAKE_ARM_SERVO_MOTOR = "intakeArmServo"

private const val SLIDE_LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0
private const val SLIDE_LIFT_COLLAPSED = 0.0 * SLIDE_LIFT_TICKS_PER_MM
private const val SLIDE_LIFT_SCORING_IN_LOW_BASKET = 806.52 * SLIDE_LIFT_TICKS_PER_MM
private const val SLIDE_LIFT_SCORING_IN_HIGH_BASKET = 1289.12 * SLIDE_LIFT_TICKS_PER_MM

private const val INTAKE_SLIDE_SERVO_START_POSITION = 0.035
private const val INTAKE_SLIDE_SERVO_END_POSITION = 0.28
private const val INTAKE_SLIDE_SERVO_POSITION_INTERVAL = 0.05

private const val INTAKE_ARM_SERVO_START_POSITION = 0.0
private const val INTAKE_ARM_SERVO_END_POSITION = 0.25
private const val INTAKE_ARM_SERVO_POSITION_INTERVAL = 0.05
private const val TELEMETRY_KEY_ROTATIONS = "Rotations"
private const val TELEMETRY_KEY_SPEED = "Speed"
private const val TELEMETRY_KEY_TRIGGER = "Trigger"
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

    private var slideLiftPosition: Double = SLIDE_LIFT_COLLAPSED

    private val intakeSlideServo: Servo by lazy {
        hardwareMap.servo.get(HARDWARE_MAP_INTAKE_SLIDE_SERVO_MOTOR)
    }

    private val intakeArmServo: Servo by lazy {
        hardwareMap.servo.get(HARDWARE_MAP_INTAKE_ARM_SERVO_MOTOR)
    }
    private var intakeArmServoPosition: Double = INTAKE_ARM_SERVO_START_POSITION

    override fun runOpMode() {
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        frontRightMotor.direction = DcMotorSimple.Direction.FORWARD
        backRightMotor.direction = DcMotorSimple.Direction.FORWARD

        // Setting zeroPowerBehavior to BRAKE enables a "brake mode".
        // This causes the motor to slow down much faster when it is coasting.
        // This creates a much more controllable drivetrain. As the robot
        // stops much quicker.
        frontLeftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        frontRightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backLeftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backRightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // Initialize the slide motor to zero
        slideMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        slideMotor.direction = DcMotorSimple.Direction.REVERSE
        slideMotor.targetPosition = 0
        slideMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        slideMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER


        intakeArmServo.position = intakeArmServoPosition

        intakeSlideServo.direction = Servo.Direction.REVERSE
        intakeSlideServo.position = INTAKE_SLIDE_SERVO_START_POSITION

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready")
        telemetry.update()

        waitForStart()

        if (isStopRequested) return

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
            if (gamepad1.y) {
                slideLiftPosition += 2800 //* cycletime
            } else if (gamepad1.a) {
                slideLiftPosition -= 2800 //* cycletime
            }

            // HERE WE CHECK TO SEE IF THE LIFT IS TRYING TO GO HIGHER THAN
            // THE MAXIMUM EXTENSION. IF IT IS, WE SET THE VARIABLE TO THE MAX.
            if (slideLiftPosition > SLIDE_LIFT_SCORING_IN_HIGH_BASKET) {
                slideLiftPosition = SLIDE_LIFT_SCORING_IN_HIGH_BASKET
            }
            //S AME AS ABOVE, WE SEE IF THE LIFT IS TRYING TO GO BELOW 0,
            // AND IF IT IS, WE SET IT TO 0.
            if (slideLiftPosition < 0) {
                slideLiftPosition = 0.0
            }

            extendVerticalSlide(slideLiftPosition.toInt())
            // END SET SLIDE MOTOR MODE AND POWER

            telemetry.addData("Slide lift position", slideLiftPosition)
            telemetry.addData("Slide target position", slideMotor.targetPosition)
            telemetry.addData("Slide current position", slideMotor.currentPosition)
            telemetry.addData("slideMotor current:", slideMotor.getCurrent(CurrentUnit.AMPS))
            // END GET CURRENT SLIDE STATE AND SET SLIDE MOTOR MODE AND POWER

            // START SET INTAKE SLIDE SERVO MOTOR POSITION
            if (gamepad1.dpad_left) {
                intakeSlideServo.position = INTAKE_SLIDE_SERVO_END_POSITION
            } else if (gamepad1.dpad_right) {
                intakeSlideServo.position = INTAKE_SLIDE_SERVO_START_POSITION
            }

            telemetry.addData("Intake Slide Servo Position", intakeSlideServo.position)
            // END SET INTAKE SLIDE SERVO MOTOR POSITION

            // START SET INTAKE ARM SERVO MOTOR POSITION
            if (gamepad1.x) {
                intakeArmServo.position = INTAKE_ARM_SERVO_START_POSITION
            } else if (gamepad1.b) {
                intakeArmServo.position = INTAKE_ARM_SERVO_END_POSITION
            }

            telemetry.addData("Intake Arm Servo Position", intakeArmServo.position)
            // END SET INTAKE ARM SERVO MOTOR POSITION

            // ADD TELEMETRY DATA AND UPDATE
            telemetry.addData(TELEMETRY_KEY_ROTATIONS, frontLeftMotor.currentPosition)
            telemetry.addData(TELEMETRY_KEY_SPEED, frontLeftMotor.power)
            telemetry.addData(TELEMETRY_KEY_TRIGGER, gamepad1.right_trigger)
            telemetry.addData(TELEMETRY_KEY_Y_VALUE, leftStickY)
            telemetry.update()
        }
    }

    fun extendVerticalSlide(verticalSlideExtendPos: Int) {
        slideMotor.targetPosition = verticalSlideExtendPos // the position you want the slides to reach
//        slideMotor.targetPositionTolerance = 1 // set accuracy to 1 tick
        slideMotor.velocity = 2100.0
        slideMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

}