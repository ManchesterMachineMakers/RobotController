package org.firstinspires.ftc.teamcode.diagnostics.tests

import com.rutins.aleks.diagonal.describe
import org.firstinspires.ftc.teamcode.LinearSlide
import org.firstinspires.ftc.teamcode.diagnostics.DiagnosticsOpMode

fun linearSlideTest(opMode: DiagnosticsOpMode) = describe<LinearSlide> { slides ->
    fun wait() {
        while (slides.isBusy() && opMode.opModeIsActive());
    }
    it("runs to base") {
        slides.goTo(slides.base)
        wait()
    }
    it("runs to cone") {
        slides.goTo(slides.toCone)
        wait()
    }
    it("runs to low pole") {
        slides.goTo(slides.low)
        wait()
    }
    it("runs to medium pole") {
        slides.goTo(slides.mid)
        wait()
    }
    it("runs to high pole") {
        slides.goTo(slides.high)
        wait()
    }
    it("goes back to the bottom so we don't break it next time") {
        slides.goTo(slides.base)
        wait()
    }
}