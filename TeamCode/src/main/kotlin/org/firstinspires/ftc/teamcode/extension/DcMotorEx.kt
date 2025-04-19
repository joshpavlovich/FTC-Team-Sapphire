package org.firstinspires.ftc.teamcode.extension

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction

fun DcMotorEx.initializeForRunToPosition(
    position: Double,
    direction: Direction,
    setPowerToZero: Boolean = false
) {
    zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    this.direction = direction
    targetPosition = position.toInt()
    mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    mode = DcMotor.RunMode.RUN_TO_POSITION
    if (setPowerToZero) {
        power = 0.0
    }
}

fun DcMotorEx.runToPosition(position: Double, velocity: Double) {
    targetPosition = position.toInt() // the position you want to reach
    this.velocity = velocity
    mode = DcMotor.RunMode.RUN_TO_POSITION
}