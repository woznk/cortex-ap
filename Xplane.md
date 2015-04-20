# Using X Plane for HIL simulation #

https://cortex-ap.googlecode.com/svn/wiki/Images/XPlane.JPG

  * **X Plane setting** (source: www.skylabs.com)

> start X Plane

> open the **operations & warnings** window

> set the **flight model per frame** value to **2**

> open the **setting / joystick, keys equipment** window

> select the **center** label

> set the three bars on the right to **0%** (**full left to most realism**)

> move the nullzone bar (at the lower section of the window) to about 10%

> open the **setting / internet connections** window

> open the **advanced** tab, check the box **IP ofdata receiver**

> digit **127.0.01** in the leftmost text box

> digit **49005** in the rightmost text box

> open the **UDP port tab**

> in the box **port that we receive on** select **49000**

> in the box **port that we send on** select **49001**

> in the box **port that we send to iPad on** select **49002**

https://cortex-ap.googlecode.com/svn/wiki/Images/apsim.JPG

  * **ApSim setting**

> start [Apsim.exe](http://code.google.com/p/cortex-ap/downloads/detail?name=ApSim.exe&can=2&q=)

> in the **Simulator** group:

> in the **IP** text box digit **127.0.0.1**

> in the **Port** text box digit **49005**

> in the **Telemetry** group:

> select the serial port that the program will use to send / receive telemetry data to Cortex AP board

> select the baud rate for telemetry data

> in the **GPS** group:

> select the serial port that the program will use to send GPS data to Cortex AP board

> select the baud rate for GPS data

  * **Cortex AP board Setup**

> connect the board's GPS port to the PC with an FTDI cable

> connect the board's telemetry port to the PC with an FTDI cable

> the board shall turn on

  * **Starting all**

> start python script attitude\_socket.py

> make sure the aircraft in X Plane is leveled

> in the ApSim program, click on **Start** button

> press the reset button on Cortex AP board
> (the board will calibrate the internal reference with the signal coming from the simulator)

> take off

> when ready, flip the **Mode** switch of the RC either to **Stabilize** or to **Auto**

> adjust PID settings

> flip the **Mode** switch of the RC back to **Manual** and then again to **Stabilize** or **Auto**
> (this resets the integral parts of the PID loops)