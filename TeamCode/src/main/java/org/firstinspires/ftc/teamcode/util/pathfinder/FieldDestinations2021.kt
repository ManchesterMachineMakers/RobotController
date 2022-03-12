package org.firstinspires.ftc.teamcode.util.pathfinder

enum class FieldDestinations2021(name: String, destX: Float, destY: Float) {
    // Do not change these without first consulting the spreadsheet, please!
    SharedHub("Shared Shipping Hub", 1219.2F, 0F),
    BlueHub("Alliance Shipping Hub (Blue)", -304.8F, 609.6F),
    RedHub("Alliance Shipping Hub (Red)", -304.8F, -609.6F),
    BlueWarehouse("Alliance Warehouse (Blue)", 1219.2F, 1219.2F),
    RedWarehouse("Alliance Warehouse (Red)", 1219.2F, -1219.2F),
    BlueStart1("Blue Starting Location 1", 304.8F, 1828.8F),
    RedStart1("Red Starting Location 1", 304.8F, -1828.8F),
    BlueStart2("Blue Starting Location 2", -914.4F, 1828.8F),
    RedStart2("Red Starting Location 2", -914.4F, -1828.8F),
    BlueStorage("Blue Storage Unit", -1524F, 914.4F),
    RedStorage("Red Storage Unit", -1524F, -914.4F),
    BlueCarousel("Blue Carousel", -1676.4F, 1676.4F),
    RedCarousel("Red Carousel", -1676.4F, -1676.4F),
    BlueBarcode1("Blue Barcode 1", 304.8F, 914.4F),
    BlueBarcode2("Blue Barcode 2", -914.4F, 914.4F),
    RedBarcode1("Red Barcode 1", 304.8F, -914.4F),
    RedBarcode2("Red Barcode 2", -914.4F, -914.4F),
    ;

    val destination: Destination = Destination(name, destX, destY)
}