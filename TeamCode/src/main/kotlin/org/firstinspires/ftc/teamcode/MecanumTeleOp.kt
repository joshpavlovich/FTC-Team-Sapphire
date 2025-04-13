package org.firstinspires.ftc.teamcode

import android.graphics.Color
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.extension.initializeForRunToPosition
import org.firstinspires.ftc.teamcode.extension.runToPosition

private const val HARDWARE_MAP_BLINKEN_LED_DRIVER = "blinkenLedDriver"
private const val HARDWARE_MAP_BUCKET_SERVO_MOTOR = "bucketServo"
private const val HARDWARE_MAP_COLOR_SENSOR = "colorSensor"
private const val HARDWARE_MAP_INTAKE_SLIDE_SERVO_MOTOR = "intakeSlideServo"
private const val HARDWARE_MAP_INTAKE_ARM_MOTOR = "intakeArmMotor"
private const val HARDWARE_MAP_INTAKE_LEFT_SERVO_MOTOR = "intakeLeftServo"
private const val HARDWARE_MAP_INTAKE_RIGHT_SERVO_MOTOR = "intakeRightServo"
private const val HARDWARE_MAP_SLIDE_MOTOR = "slideMotor"

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
private const val SLIDE_LIFT_LEVEL_ONE_ASCENT = 215.9 * SLIDE_LIFT_TICKS_PER_MM
private const val SLIDE_LIFT_SCORING_IN_HIGH_BASKET = 976.0 * SLIDE_LIFT_TICKS_PER_MM
private const val SLIDE_LIFT_VELOCITY = 2100.0

private const val BUCKET_SERVO_INIT_POSITION = 0.0
private const val BUCKET_SERVO_START_POSITION = 0.20
private const val BUCKET_SERVO_END_POSITION = 0.50

private const val INTAKE_SLIDE_SERVO_START_POSITION = 0.0
private const val INTAKE_SLIDE_SERVO_END_POSITION = 0.28

private const val INTAKE_ARM_START_POSITION = 2.35 * ARM_MOTOR_TICKS_PER_MM
private const val INTAKE_ARM_END_POSITION = 47.3 * ARM_MOTOR_TICKS_PER_MM
private const val INTAKE_ARM_LOW_CHAMBER_SCORING_POSITION = 31 * ARM_MOTOR_TICKS_PER_MM
private const val INTAKE_ARM_VELOCITY = 800.0

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

    private val intakeLeftServo: CRServo by lazy {
        hardwareMap.crservo.get(HARDWARE_MAP_INTAKE_LEFT_SERVO_MOTOR)
    }

    private val intakeRightServo: CRServo by lazy {
        hardwareMap.crservo.get(HARDWARE_MAP_INTAKE_RIGHT_SERVO_MOTOR)
    }

    private val colorSensor: NormalizedColorSensor by lazy {
        hardwareMap.colorSensor.get(HARDWARE_MAP_COLOR_SENSOR) as NormalizedColorSensor
    }

    private val blinkinLedDriver: RevBlinkinLedDriver by lazy {
        hardwareMap.get(RevBlinkinLedDriver::class.java, HARDWARE_MAP_BLINKEN_LED_DRIVER)
    }

    // Once per loop, we will update this hsvValues array. The first element (0) will contain the
    // hue, the second element (1) will contain the saturation, and the third element (2) will
    // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
    // for an explanation of HSV color.
    val hsvValues: FloatArray = FloatArray(3)

    override fun runOpMode() {
        // By setting these values to new Gamepad(), they will default to all
        // boolean values as false and all float values as 0
        val currentGamepad1 = Gamepad()
        val previousGamepad1 = Gamepad()

        robot.initialize()

        slideMotor.initializeForRunToPosition(SLIDE_LIFT_COLLAPSED, Direction.REVERSE, true)

        // Initialize the arm motor to zero
        intakeArmMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intakeArmMotor.direction = Direction.FORWARD
        intakeArmMotor.targetPosition = 0
        intakeArmMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        intakeArmMotor.mode = DcMotor.RunMode.RUN_TO_POSITION

        intakeArmMotor.runToPosition(INTAKE_ARM_START_POSITION, INTAKE_ARM_VELOCITY)

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
            // Store the gamepad values from the previous loop iteration in
            // previousGamepad1/2 to be used in this loop iteration.
            // This is equivalent to doing this at the end of the previous
            // loop iteration, as it will run in the same order except for
            // the first/last iteration of the loop.
            previousGamepad1.copy(currentGamepad1)

            // Store the gamepad values from this loop iteration in
            // currentGamepad1/2 to be used for the entirety of this loop iteration.
            // This prevents the gamepad values from changing between being
            // used and stored in previousGamepad1/2.
            currentGamepad1.copy(gamepad1)

            val armIntakeDown = intakeArmMotor.currentPosition > 900
            val intakeArmDownPowerReducer: Double = if (armIntakeDown) 1.0 else 0.0
            // Remember, Y stick value is reversed
            robot.move(
                axial = -currentGamepad1.left_stick_y.toDouble(),
                lateral = currentGamepad1.left_stick_x.toDouble() * 1.1,
                yaw = currentGamepad1.right_stick_x.toDouble(),
                powerMultiplier = currentGamepad1.left_trigger.toDouble(),
                powerReducer = if (armIntakeDown) intakeArmDownPowerReducer else currentGamepad1.right_trigger.toDouble()
            )

            // START SET SLIDE MOTOR MODE AND POWER
            if (currentGamepad1.dpad_up) {
                slideMotor.runToPosition(SLIDE_LIFT_SCORING_IN_HIGH_BASKET, SLIDE_LIFT_VELOCITY)
            } else if (currentGamepad1.dpad_down) {
                slideMotor.runToPosition(SLIDE_LIFT_COLLAPSED, SLIDE_LIFT_VELOCITY)
            } else if (currentGamepad1.guide) {
                slideMotor.runToPosition(SLIDE_LIFT_LEVEL_ONE_ASCENT, SLIDE_LIFT_VELOCITY)
            }

            if (!slideMotor.isBusy && slideMotor.targetPosition <= 0) {
                telemetry.addData("Slide not busy", slideMotor.currentPosition)
                slideMotor.initializeForRunToPosition(SLIDE_LIFT_COLLAPSED, Direction.REVERSE, true)
            }
            // END SET SLIDE MOTOR MODE AND POWER

            telemetry.addData("Slide motor target position", slideMotor.targetPosition)
            telemetry.addData("Slide motor current position", slideMotor.currentPosition)
            // END GET CURRENT SLIDE STATE AND SET SLIDE MOTOR MODE AND POWER

            // START SET ARM MOTOR MODE AND POWER
            if (currentGamepad1.triangle) {
                intakeArmMotor.runToPosition(INTAKE_ARM_END_POSITION, INTAKE_ARM_VELOCITY)
            } else if (currentGamepad1.cross) {
                intakeArmMotor.runToPosition(INTAKE_ARM_START_POSITION, INTAKE_ARM_VELOCITY)
            } else if (currentGamepad1.touchpad) {
                intakeArmMotor.runToPosition(
                    INTAKE_ARM_LOW_CHAMBER_SCORING_POSITION,
                    INTAKE_ARM_VELOCITY
                )
            }
            // END SET ARM MOTOR MODE AND POWER

            telemetry.addData("Intake arm motor target position", intakeArmMotor.targetPosition)
            telemetry.addData("Intake arm motor current position", intakeArmMotor.currentPosition)
            // END GET CURRENT SLIDE STATE AND SET SLIDE MOTOR MODE AND POWER

            // START SET BUCKET SERVO MOTOR POSITION
            if (currentGamepad1.square) {
                bucketServo.position = BUCKET_SERVO_END_POSITION
            } else if (currentGamepad1.circle) {
                bucketServo.position = BUCKET_SERVO_START_POSITION
            }

            telemetry.addData("Bucket Servo Position", bucketServo.position)
            // END SET BUCKET SERVO MOTOR POSITION

            // START SET INTAKE SLIDE SERVO MOTOR POSITION
            if (currentGamepad1.dpad_left) {
                intakeSlideServo.position = INTAKE_SLIDE_SERVO_END_POSITION
            } else if (currentGamepad1.dpad_right) {
                intakeSlideServo.position = INTAKE_SLIDE_SERVO_START_POSITION
            }

            telemetry.addData("Intake Slide Servo Position", intakeSlideServo.position)
            // END SET INTAKE SLIDE SERVO MOTOR POSITION

            // START SET INTAKE SERVO POWER
            if (currentGamepad1.left_bumper) {
                intakeLeftServo.direction = Direction.FORWARD
                intakeRightServo.direction = Direction.REVERSE
            }

            if (currentGamepad1.right_bumper) {
                intakeLeftServo.direction = Direction.REVERSE
                intakeRightServo.direction = Direction.FORWARD
            }

            val intakeServoPower = if (currentGamepad1.left_bumper || currentGamepad1.right_bumper) {
                1.0
            } else {
                0.0
            }
            intakeLeftServo.power = intakeServoPower
            intakeRightServo.power = intakeServoPower
            // END SET INTAKE SERVO POWER

            // START COLOR SENSOR
            // Get the normalized colors from the sensor
            val colors: NormalizedRGBA = colorSensor.normalizedColors
            // Update the hsvValues array by passing it to Color.colorToHSV()
            Color.colorToHSV(colors.toColor(), hsvValues)
            val hue = hsvValues[0].toInt()
            val pattern = when (hue) {
                in 20..30 -> RevBlinkinLedDriver.BlinkinPattern.RED
                in 65..85 -> RevBlinkinLedDriver.BlinkinPattern.YELLOW
                in 200..240 -> RevBlinkinLedDriver.BlinkinPattern.BLUE
                else -> RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE
            }
            blinkinLedDriver.setPattern(pattern)
            // END COLOR SENSOR

            telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2])
            telemetry.addData("Alpha", "%.3f", colors.alpha)

            // ADD TELEMETRY DATA AND UPDATE
            telemetry.update()
        }
    }
}