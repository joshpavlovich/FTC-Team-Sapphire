package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.extension.initializeForRunToPosition
import org.firstinspires.ftc.teamcode.extension.runToPosition

private const val HARDWARE_MAP_SLIDE_MOTOR = "slideMotor"
private const val HARDWARE_MAP_BUCKET_SERVO_MOTOR = "bucketServo"
private const val HARDWARE_MAP_INTAKE_SLIDE_SERVO_MOTOR = "intakeSlideServo"
private const val HARDWARE_MAP_INTAKE_ARM_MOTOR = "intakeArmMotor"
private const val HARDWARE_MAP_INTAKE_SERVO_MOTOR = "intakeServo"

// Encoder Resolution for Viper Slide 223 RPM Motor = ((((1+(46/11))) * (1+(46/11))) * 28)
// From https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-26-9-1-ratio-24mm-length-8mm-rex-shaft-223-rpm-3-3-5v-encoder/
// Encoder Resolution for Arm 84 RPM Motor = ((((1+(46/17))) * (1+(46/17))) * (1+(46/11)) * 28)
// From https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-71-2-1-ratio-24mm-length-8mm-rex-shaft-84-rpm-3-3-5v-encoder/
// Ticks Per Revolution = (Motor Encoder Resolution / Diameter Millimeters)
private const val SLIDE_LIFT_TICKS_PER_MM =
    (751.8) / 120 // Encoder Resolution Formula ->	((((1+(46/11))) * (1+(46/11))) * 28) = 751.8
private const val ARM_MOTOR_TICKS_PER_MM =
    (1992.6) / 96.0 // Encoder Resolution Formula ->	((((1+(46/17))) * (1+(46/17))) * (1+(46/11)) * 28) = 1992.6

// Distance in Millimeters for High Basket scoring position = high basket height in Millimeters * Viper Slide Lift Ticks Per Millimeter
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

@TeleOp(name = "Team Sapphire: Mecanum TeleOp", group = "Robot")
class MecanumTeleOp : LinearOpMode() {

    // Instance of the "Robot" class.
    private val robot = BaseMecanumRobot(this)

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
        robot.initialize()

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
            // Remember, Y stick value is reversed
            robot.move(
                axial = -gamepad1.left_stick_y.toDouble(),
                lateral = gamepad1.left_stick_x.toDouble() * 1.1,
                yaw = gamepad1.right_stick_x.toDouble(),
                powerMultiplier = gamepad1.left_trigger.toDouble()
            )

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
            telemetry.update()
        }
    }
}