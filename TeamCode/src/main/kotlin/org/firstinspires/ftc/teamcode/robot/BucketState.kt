package org.firstinspires.ftc.teamcode.robot

private const val BUCKET_SERVO_INIT_POSITION = 0.0
private const val BUCKET_SERVO_UP_POSITION = 0.20
private const val BUCKET_SERVO_DOWN_POSITION = 0.50

enum class BucketState(val position: Double) {
    INIT(BUCKET_SERVO_INIT_POSITION),
    UP(BUCKET_SERVO_UP_POSITION),
    DOWN(BUCKET_SERVO_DOWN_POSITION)
}