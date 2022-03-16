package org.firstinspires.ftc.teamcode.util.pathfinder

enum class FieldDestinations2021(name: String, destX: Float, destY: Float, clearanceRadius: Float) {
    // Do not change these without first consulting the spreadsheet, please!
    SharedHub("Shared Shipping Hub", 1219.2F, 0F, 1557.2F),
    BlueHub("Alliance Shipping Hub (Blue)", -304.8F, 609.6F, 1557.2F),
    RedHub("Alliance Shipping Hub (Red)", -304.8F, -609.6F, 1557.2F),
    BlueWarehouse("Alliance Warehouse (Blue)", 1219.2F, 1219.2F, 1100.0F),
    RedWarehouse("Alliance Warehouse (Red)", 1219.2F, -1219.2F, 1100.0F),
    BlueStart1("Blue Starting Location 1", 304.8F, 1828.8F, 1404.8F),
    RedStart1("Red Starting Location 1", 304.8F, -1828.8F, 1404.8F),
    BlueStart2("Blue Starting Location 2", -914.4F, 1828.8F, 1404.8F),
    RedStart2("Red Starting Location 2", -914.4F, -1828.8F, 1404.8F),
    BlueStorage("Blue Storage Unit", -1524F, 914.4F, 1404.8F),
    RedStorage("Red Storage Unit", -1524F, -914.4F, 1404.8F),
    BlueCarousel("Blue Carousel", -1676.4F, 1676.4F, 1100.0F),
    RedCarousel("Red Carousel", -1676.4F, -1676.4F, 1100.0F),
    BlueBarcode1("Blue Barcode 1", 304.8F, 914.4F, 1100.0F),
    BlueBarcode2("Blue Barcode 2", -914.4F, 914.4F, 1100.0F),
    RedBarcode1("Red Barcode 1", 304.8F, -914.4F, 1100.0F),
    RedBarcode2("Red Barcode 2", -914.4F, -914.4F, 1100.0F),
    ;

    val destination: Destination = Destination(name, destX, destY, clearanceRadius)
}