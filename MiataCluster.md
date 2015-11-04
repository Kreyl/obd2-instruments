Information on driving Mazda Miata instrument cluster gauges and indicators.

# Introduction #

This page contains information on using a microcontroller to operate the instrument cluster gauges on a Mazda Miata.

We compiled this information as part of our EV Miata project.  We designed a motor controller and gauge driver controller.  They communicate using CAN bus and support OBD2 diagnostics and control.


# Details #

Gauges to drive

| **Connector** | **Color** | **Gauge** | **EV Use** |
|:--------------|:----------|:----------|:-----------|
| 1M            | Y/L       | Tachometer | current draw or motor speed |
| 2L            | B/L       | Water temperature | heatsink temperature |
| 2B            | Y/R       | Oil pressure | motor temperature or freewheel ratio |
| 1A            | Y         | Fuel level | battery charge level |
| 2J            | B         | gauge ground|

Indicator lights to drive, all switched to ground to turn on

| Check Engine  | 1.4W |
|:--------------|:-----|
| Charge	       | 1.4W |
| OD off	       | 1.4W |

Possible cluster outputs

Speed sensor (from speedometer?) on 2F

Notes on the cluster connector
| 2J | B | gauge ground|
|:---|:--|:------------|
| 2K | B/Y |12V switched supply|
| 1M | Y/L | Tach signal | from ignitor|
| 1A | Y  |  Fuel gauge |
| 2L | B/L | Water temperature | resistor to ground|
| 2B | Y/R | Oil pressure gauge |current to ground|
| 1C | Y/B | Engine check indicator |switch to ground|
| 1G | W/B | Charge indicator |pilot current to ground|


Engine Compartment Connectors
> [do: Locate, describe and label connectors ](To.md)

Current measurements

300mA-370mA base current draw on the +12V supply