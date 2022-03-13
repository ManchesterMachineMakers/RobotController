package org.firstinspires.ftc.teamcode.diagnostics.util

object KTestable {
    inline fun <reified T : Testable> get(arr: Array<out Testable>?): T {
        return Testable.get(arr, T::class.java)
    }
    inline fun <reified T : Testable> getOrDie(arr: Array<out Testable>?): T {
        return Testable.getOrDie(arr, T::class.java)
    }
}