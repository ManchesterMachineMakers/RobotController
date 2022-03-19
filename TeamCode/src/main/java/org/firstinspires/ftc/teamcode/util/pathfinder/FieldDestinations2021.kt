package org.firstinspires.ftc.teamcode.util.pathfinder

enum class FieldDestinations2021(name: String, destX: Float, destY: Float, clearanceRadius: Float) {
    // Do not change these without first consulting the spreadsheet, please!
    // https://docs.google.com/spreadsheets/d/1AovRhnWJCv5u5fJtydj7_rNhSQgOsxTw/edit?usp=sharing&ouid=104857601293887335871&rtpof=true&sd=true
    //
    SharedHub("Shared Shipping Hub", 1219.2F, 0F, 1785.8F),
    BlueHub("Alliance Shipping Hub (Blue)", -304.8F, 609.6F, 1785.8F),
    RedHub("Alliance Shipping Hub (Red)", -304.8F, -609.6F, 1785.8F),
    BlueWarehouse("Alliance Warehouse (Blue)", 1219.2F, 1219.2F, 1328.6F),
    RedWarehouse("Alliance Warehouse (Red)", 1219.2F, -1219.2F, 1328.6F),
    BlueStart1("Blue Starting Location 1", 304.8F, 1828.8F, 1633.4F),
    RedStart1("Red Starting Location 1", 304.8F, -1828.8F, 1633.4F),
    BlueStart2("Blue Starting Location 2", -914.4F, 1828.8F, 1633.4F),
    RedStart2("Red Starting Location 2", -914.4F, -1828.8F, 1633.4F),
    BlueStorage("Blue Storage Unit", -1524F, 914.4F, 1633.4F),
    RedStorage("Red Storage Unit", -1524F, -914.4F, 1633.4F),
    BlueCarousel("Blue Carousel", -1676.4F, 1676.4F, 1328.6F),
    RedCarousel("Red Carousel", -1676.4F, -1676.4F, 1328.6F),
    BlueBarcode1("Blue Barcode 1", 304.8F, 914.4F, 1328.6F),
    BlueBarcode2("Blue Barcode 2", -914.4F, 914.4F, 1328.6F),
    RedBarcode1("Red Barcode 1", 304.8F, -914.4F, 1328.6F),
    RedBarcode2("Red Barcode 2", -914.4F, -914.4F, 1328.6F),
    ;

    val destination: Destination = Destination(name, destX, destY, clearanceRadius)
}