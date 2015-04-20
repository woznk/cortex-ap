# Glossary #

**AGL** Altitude above ground level

**AHRS** Attitude and Heading Reference System. Result of computations made on raw sensor data coming from IMU to establish a plane's XYZ and heading orientation.

**BEC** Battery Elimination Circuit. Provides constant voltage for RC equipment and other onboard electronics.

**DCM** Direction Cosine Matrix. One possible representation of the attitude of the aircraft. Another representation is Euler angles.

**ESC** Electronic Speed Control. Device to control the motor in an electric aircraft. May include a BEC, which provides power for the RC system and other onboard electronics.

**FPV** First-person view. Consists in piloting a model aircraft as if the pilot was onboard the aircraft itself. Uses an onboard video camera and wireless transmission to send live video to the pilot that stays on the ground and looks in to a monitor or wears video goggles.

**FTDI** USB to serial converter. Available either as a chip, or as a ready to use cable. FTDI is the brand name of the company that makes it.

**GCS** Ground Control Station. Software running on a computer on the ground that receives telemetry information from the aircraft and displays its status. May include video and other sensor data. Can also be used to transmit in-flight commands.

**HIL** Hardware in the loop. Flight simulation software running on a computer generates data that simulates the data that would be coming from an autopilot's sensors. The autopilot is running and doesn't "know" that the data is simulated, so it responds just as it would to real sensor data. Hardware-in-the-loop uses the physical autopilot hardware connected to a simulator, as opposed to simulating the autopilot in software, too. In an "open loop" simulation, the software simulator feeds data to the hardware autopilot; in a "closed loop" simulation, the hardware feeds data back to the sofware simulator, too.

**I2C** Synchronous serial protocol and bus invented by Philips (now NXP). Allows to connect multiple devices as sensors, I/O, etc. to a microcontroller with only two wires.

**IMU** Inertial measurement unit. Consists of accelerometers and gyrometers. Accelerometer measures the component of gravity along its axis, gyrometer measures rotation around its axis. Accelerometers are influenced by aircraft accelerations but their signal is reliable in the long term, gyrometers drift over time.

**INS** Inertial Navigation System. A system thar infers aircraft position based on GPS position, attitude and speed sensors. The technique is called "ded(uced) reckoning".

**Kalman Filter** Algorithm that combines signals with complementary statistical features as those coming from accelerometer and gyrometers to obtain a better estimate of the system (aircraft) status (attitude).

**LOS** Line of Sight.

**LiPo** Lithium Polymer battery. Higher power to weight ratio compared to NiMh batteries.

**NMEA** National Marine Electronics Association standard for GPS information.

**OSD** On-screen display. Overlay of informations to the real-time video stream.

**PID** Proportional/Integral/Derviative control method. A machine control algorithm that allows for more accurate sensor-motion control loops and less overcontrol.

**RTL** Return to Launch. Launch is equivalent to Home location where the aircraft took off.

**SVN** Subversion version-control system used by this project for source code.

**UAV** Unmanned Aerial Vehicle.

**VLOS** Visual Line of Sight. The pilot's ability to see an aircraft from the ground without the use of artificial visual aids.

**WAAS** Wide Area Augmentation System. A system of satellites and ground stations that provide GPS signal corrections, giving up to five times better position accuracy than uncorrected GPS.