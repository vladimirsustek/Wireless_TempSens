<h2>The BleuPill tempSens!</h2>

<h2>Intended usages</h2>
<ol>
  <li>Battery operating low-power temperature sensor</li>
  <li>PID controlled loop (PWM H-Bridge)</li>
</ol>

System consist of: <br> 
<ol>
  <li>Core MCU - STM32F103C8T6 BluePill module</li>
  <li>nRF24L01+ wireless module</li>
  <li>LM75AD temperature sensor</li>
  <li>2x NTC 10k on-board sensor</li>
  <li>Optional PWM H-Bridge DRV8220</li>
  <li>Optional self-measuring current consumption circuit (OA LMV358 1/2 sensing 10Ohm) - curr</li>
  <li>Optional OA channel sensing general purpose external voltage (LMV358 2/2) - oagp</li>
</ol>

This project has default structure for STM's [STM32CudeIDE](https://www.st.com/en/development-tools/stm32cubeide.html#:~:text=STM32CubeIDE%20is%20an%20advanced%20C,and%20GDB%20for%20the%20debugging.)
.Download this free IDE to build the project.<br/>
**The project utilizes default STM's HAL drivers**<br/>

To see complete PCB design, see "PCB_design" directory, or at least the [schematic](/PCB_design/Wireless_TempSens.pdf)

<h2>Battery operating low-power temperature sensor</h2>

System shall periodically send all measured data in specified interval.<br/>
System shall consume minimal power when not measuring or sending the data .<br/>
System shall be parametrized over the UART (115200b, LF terminated):
<ol>
  <li>nRF24L01+'s TX_ADR: 5 - bytes e.g. ADR01</li>
  <li>nRF24L01+'s RX_ADR_P0: 5 - bytes e.g. ADR01</li>
  <li>PERIOD INTERVAL: number 0 - 255 representing low power mode in seconds</li>
</ol>
Note that the UART connection is meant as optional for parametrization and logging.

<h3>BluePill modifications</h3>
Due to the permanetly on PWR LED (when connected to the 3V3)<br/>
and the PC13 LED (on due to the TAMPER RTC-ALARM PIN),<br/>
their pull-up resistors has been desoldered (R5, R1).<br/>
Note that each LED consumed cca 2mA what is inadmissible for low power mode.


<h3>Startup and operational sequence<br/></h3>
After reset, the application sends UART messages:<br/><br/>
<em>LISTENING FOR PARAMETRIZATION 10000 ms<br/>
FORMAT: ADR01 ADR01 010</em><br/>...displaying currently set parameters<br/><br/>
When a new parameters has been received and set, they are displayed:<br/><br/>
<em>TX_ADR ADR01<br/>
RX_ADR ADR01<br/>
STOP_PERIOD 58<br/>
NORMAL MODE STARTED</em><br/>
When no or invalid parameters were provided:<br/><br/>
<em>NO CHANGES APPLIED<br/>
NORMAL MODE STARTED</em/>

Then, once per pre-set interval the system sends over the UART message:<br/><br/>
<em>TMPE 23750 - the temperature by the LM75AD [m°C]<br/>
NTC2 1334 - the NTC2 measured voltage [mV]<br/>
NTC1 1392  - the NTC1 measured voltage [mV]<br/>
NTC2 8998 - the NTC2 calculated resistance [mV]<br/> 
NTC1 9768 - the NTC1 calculated voltage [mV]<br/>
NTC2 274344 - the NTC2 interpolated temperature [m°C]<br/>
NTC1 255636 - the NTC1 interpolated temperature [m°C]<br/>
OAGP 1236 - the measured voltage on the oagp [mV]<br/>
CURR 1140 - the measured voltage on the curr [mV]<br/>
TMPI 34 - the MCU internal sensor temperature [°C]<br/>
VREF 2815 - the MCU power supply VDDA [mV]<br/>
NRF STATUS: 0x1e - the STATUS register of the nRF24L01</em><br/>

And also sends over the nRF24L01+ structure consist of (wireless):<br/><br/>
<em>int32_t ntc2;<br/>
int32_t ntc1;<br/>
int32_t oagp;<br/>
int32_t curr;<br/>
int32_t tmpi;<br/>
int32_t vdda;<br/>
int32_t tmpe;</em><br/>

With the same meaning like explained for the UART message.<br/>
The ntc2 and ntc1 is already the interpolated temperature value.<br/>

<h3>Low power current consumption</h3>

<h4>LP (Low Power) - STOP_MODE the deepest MCU sleep mode</h4>

According to the the [STM32F103C8T6 datasheet](/Docs/stm32f103cb.pdf) the STOP_MODE
consumption is 14uA for the VDDA = 3.3V<br>

As the BluePill utilizes the LDO [RT9193-33](/Docs/RT9193-33PB-Richtek.pdf) with
quescient current 90-130uA and all external peripherals are<br/>controlled using the
P-FET transistor with 470kOhm pull-up (7uA) - disconnected during the STOP_MODE.<br/>

The maximal STOP_MODE consumption for VDDA = 3.3V (5V BluePill power supply) is:<br/>

<em> I_LP = I_MCU_STOP_MODE + I_LDO_QUESCIENT + I_T_PULL_UP = 14uA + 130uA + 7uA = 150uA.

<h4>ON - measuring the temperatures/voltages</h4>

The On-Mode consumption mostly consist of the:<br/>

STM32CubeIDE:PCC_Tool_Calculator MCU consumption at 2MHz clock = 1.9mA<br/>
[LM75AD](/Docs/LM75A.pdf) normal mode consumption = 100uA<br/>
NTC1 + NTC2 consumption at 25°C (10k+10k) = 2x165uA = 330uA<br/>

<em> I_ON = I_MCU+ I_LM75AD + I_NTC = 1900uA + 100uA + 330uA = 2380uA</em>

<h4>TX - nRF24L01+ transmit with 16 repeat attempts, 4000us timeout</h4>

The TX mode:<br/>
nRF24L01+ consumption in TX mode (0dBm max power):

<em> I_TX = I_ON + I_NRF_TX  = 2380uA + 13000uA = 15080</em>

The maximal time spent in TX mode for payload = 32Bytes<br/>
bits_to_transmit = preamb + address + packet control field + payload + crc<br/>
bits_to_transmit = 8 + 5*8 + 9 + 32*8 + 2*8 = 329 bits<br/>

329bits are transmitted with 2Mbps in time 329/2,000,000 = 165us<br/>

However as the ACK mode is selected (the same amount might be sent back)<br/>
the duration 330us shall be taken in account (omitted other minor latencies).<br/>

To be sure, 16 re-transmits with 4000us timeout are selected.<br/>

thus I_TX might take up to cca 16x4000 = 64ms => 100ms TX time selected.<br/>

<br/>See the [nRF24L01+](nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf) for more info<br/>

<h4>Average consumption</h4>

The average current consumption for the operational sequence:

I_ON = 1.9 seconds<br/>
I_TX = 0.1 seconds<br/>
I_LP = 58  seconds<br/>

I_AVG = (1.9x2380uA + 0.1x15080 + 58x150uA)/60 = 245uA<br/>

As the conventional AA or AAA batteries are providing good power down to the 0.9V,<br/>
and all the used components quarantie operation down to the 2.7V (3x0.9V)<br/>

For the 1500mAh batteries, expected durability of operation is:

t = 1500 / I_AVG = 1500 / 0.245 = 6122 hours => t = 255 days


<h4>Evaluation - Measured consumption</h4>

For the 5V power supply, I_LP = 116uA<br/>
For the 2.9V power supply, I_LP = 82uA<br/>

<img src="/Docs/CompleteSystem.jpg" alt="Demo" title="Demo">

**Looks like the project fullfills the requirements !!**



