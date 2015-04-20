## Multi Wii protocol parameters ##

  * **Command messages**
| **"data get" commands** | **Implemented** |
|:------------------------|:----------------|
| MSP\_IDENT     | Y |
| MSP\_STATUS    | Y |
| MSP\_RAW\_IMU   | N |
| MSP\_SERVO     | N |
| MSP\_MOTOR     | N |
| MSP\_RC        | N |
| MSP\_RAW\_GPS   | N |
| MSP\_COMP\_GPS  | N |
| MSP\_ATTITUDE  | N |
| MSP\_ALTITUDE  | N |
| MSP\_BAT       | N |
| MSP\_RC\_TUNING | N |
| MSP\_PID       | Y |
| MSP\_BOX       | N |
| MSP\_MISC      | N |
| MSP\_MOTOR\_PIN | N |
| MSP\_BOXNAMES  | N |
| MSP\_PIDNAMES  | N |
| MSP\_WP        | N |
| MSP\_HEADING   | N |
| MSP\_DEBUGMSG  | N |
| MSP\_DEBUG     | N |
---
| **"data set" commands** | **Implemented** |
| MSP\_SET\_RAW\_RC        | N |
| MSP\_SET\_RAW\_GPS       | N |
| MSP\_SET\_PID           | Y |
| MSP\_SET\_BOX           | N |
| MSP\_SET\_RC\_TUNING     | N |
| MSP\_ACC\_CALIBRATION   | N |
| MSP\_MAG\_CALIBRATION   | N |
| MSP\_SET\_MISC          | N |
| MSP\_RESET\_CONF        | N |
| MSP\_WP\_SET            | N |