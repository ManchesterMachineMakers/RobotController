package org.firstinspires.ftc.teamcode.util.pathfinder

enum class FieldDestinations2021(name: String, destX: Float, destY: Float) {
    //TODO: add actual values
    SharedHub("Shared Shipping Hub", 100F, 100F),
    BlueHub("Alliance Shipping Hub (Blue)", 100F, 300F),
    RedHub("Alliance Shipping Hub (Red)", 300F, 300F),
    BlueWarehouse("Alliance Warehouse (Blue)", 100F, 200F),
    RedWarehouse("Alliance Warehouse (Red)", 300F, 200F),
    BlueStart("Blue Starting Location", 0F, 50F),
    RedStart("Red Starting Location", 300F, 50F)
    ;

    val destination: Destination = Destination(name, destX, destY)
}