#genfield
This repo started as a clone of the winch battery-to-invertor contactor control repo. See GliderWinchItems/contactor.

The hardware uses the f103Ard pcb (see GliderWinchCommons/embed/svn_sensor/hw/trunk/eagle/f103Ard) to control the generator field and throttle.

The processor is Blue Pill STM32F103 sub-board that mounts on the pcb.
The key configuration items of the board--
- Non-isolated CAN driver 
- Two Hall-effect current sensors for battery and motor currect
- Two isolated FET drivers for contactors using off-board sub-board
- Isolated high voltage measurement via isolated uart RX
- One small fet to pull invertor /enable line
