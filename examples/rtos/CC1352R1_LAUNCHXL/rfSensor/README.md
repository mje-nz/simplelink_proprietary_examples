---
# rfSensor

---

## Example Summary

This example is intended as an advance example to show how to use the RF
Driver to implement a proprietary radio protocol. The example showcase a
simple time-slotted synchronous collector and sensor protocol and is meant to
be used together with the rfCollector example.

The sensor can associate itself with a collector by responding to a beacon
using a unique sensor address. If successfull, the sensor will be given a fixed
time-slot during the collector defined beacon interval.

## Peripherals Exercised

* `Board_GPIO_LED0`  - Heartbeat LED indicating the processingTask is running

## Resources & Jumper Settings

> If you're using an IDE (such as CCS or IAR), please refer to Board.html in
your project directory for resources used and board-specific jumper settings.
Otherwise, you can find Board.html in the directory
&lt;SDK_INSTALL_DIR&gt;/source/ti/boards/&lt;BOARD&gt;.

## Example Usage

Build and download the example to the development board. When running the
example, a serial printout is available over UART, containing information on
current sensor data and connection status. The following default parameters
are used for the UART peripheral:

  UART Param     |Default Values
  -------------- |----------------
  Baud Rate      |115200
  Data Length    |8 bits
  Parity         |None
  Stop Bits      |1 bit
  Flow Control   |None

If the network consist of multiple slaves, each slave device must be given a
unique device address. The sensor address is set by the ``SENSOR_ADDRESS``
define at the top of the ``rfSensor.c`` file.

## Application Design Details

The radio protocol is setup to support N number of sensors in one network.
The number of sensors supported depends on the size of the time-slot and the
collector beacon interval. All communication between collector and sensor is
confined within the beacon interval. At the beginning of each beacon interval
the collector sends out a beacon to allow sensors to associate itself with the
network. The remaining part of the time window given by the beacon interval
time is used for communication with associated sensors.

Per default, the example uses a 2 s beacon interval and 200 ms time-slot size.
The numbers of supported nodes (N) is calculated based on this number as
such: (2000 / 200) - 1 = 9. The beacon interval and time-slot size can be
adjusted by changing the pre-defined value of BEACON_INTERVAL and TIMESLOT_SIZE
respectively. For cases where a low data rate PHY is used, for example
SimpleLink Long Range, it might be necessary to increase the TIMESLOT_SIZE.

A graphical representation of how the beacon interval is divided is shown
below:

![](C:\Workspaces\General\resources\beacon_interval_time_slot_dist.png)

The first time-slot is reserved for the collector beacon. A sensor that
listens for a beacon may respond to a beacon, requesting association with the
collector. The sequence diagram below shows the communication exchange between
a collector and sensor during an association.

The application does not implement any collision avoidance functionality. If
multiple sensors try to associate with the collector at the same time they
will most likely collide with each other in time and fail the association.

![](C:\Workspaces\General\resources\association_sequence.png)

During association, the collector will distribute a time-slot to the sensor.
The sensor will then use the time stamp provided by the Radio Core together
with the provided time-slot to schedule its report interval.

During a sensor time-slot, the collector will listen for data from the sensor
assigned to that particular time slot. When receiving a sensor packet, the
collector will acknowledge the sensor by responding with an ACK. A sequence
diagram of the complete beacon interval is seen below:

![](C:\Workspaces\General\resources\complete_sequence.png)

When expecting to receive a packet from a particular sensor, the address
filtering feature provided by the Radio Core is leveraged.

>The collector expects each sensor to be given a unique device address and the
behavior if multiple sensors have the same address is undefined.

There is still a possibility to receive "false" packets if the address field
happens to match. However, for a packet to be valid, the header also needs to
match what is expected by the application.

There is no drift compensation between the collector and sensor; over time the
collector and sensor will drift apart and lose the connection. The connection
will be re-established when both the collector and sensor consider the sensor
an orphan. A simple way of implementing drift compensation is to leverage the
Radio Core timestamp feature on both the collector and sensor and to report
the deviation to the sensor during each acknowledgment so that it can re-align.
