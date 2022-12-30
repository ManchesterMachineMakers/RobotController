package org.firstinspires.ftc.teamcode
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.ExportToBlocks
import org.firstinspires.ftc.teamcode.util.CustomBlocksOpModeCompanion

object ArmClaw : CustomBlocksOpModeCompanion() {
    private lateinit var clawServo: Servo
    private lateinit var wristServo: Servo

    override fun exists()
        = hardwareMap.servo.contains("clawServo") && hardwareMap.servo.contains("wristServo")

    override fun initHardware() {
        clawServo = hardwareMap.servo.get("clawServo")
        wristServo = hardwareMap.servo.get("wristServo")
        wristServo.scaleRange(0.22, 0.4)
    }

    @JvmStatic @ExportToBlocks
    fun open() {
        clawServo.position = 0.5
    }

    @JvmStatic @ExportToBlocks
    fun close() {
        clawServo.position = 1.0
    }

    /**
     * Set wrist position.
     * @param position new wrist position, between 0 (up) and 1 (down).
     */
    @JvmStatic @ExportToBlocks
    fun setWrist(position: Double) {
        wristServo.position = position
    }
}