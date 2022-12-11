package org.firstinspires.ftc.teamcode.diagnostics.tests

import com.rutins.aleks.diagonal.describe
import org.firstinspires.ftc.teamcode.LinearSlide
import org.firstinspires.ftc.teamcode.diagnostics.DiagnosticsOpMode

fun linearSlideTest(opMode: DiagnosticsOpMode) = describe<LinearSlide> { slides ->
    fun wait() {
        while (slides.isBusy() && opMode.opModeIsActive());
    }
    it("runs to base") {
        slides.goToBase()
        wait()
    }
    it("runs to cone") {
        slides.goToBase()
        wait()
    }
    it("runs to low pole") {
        slides.goToLow()
        wait()
    }
    it("runs to medium pole") {
        slides.goToMid()
        wait()
    }
    it("goes back to the bottom so we don't break it next time") {
        slides.goToBase()
        wait()
    }
}