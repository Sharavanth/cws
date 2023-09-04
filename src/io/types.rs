#[cfg(feature = "agas")]
use embedded_hal::blocking::delay::DelayUs;
use lcd1602::LcdI2c;
use max6675::Max6675;
use muxhc4067::Mux74hc4067;
#[cfg(feature = "agas")]
use pid::Pid;
use stm32f1xx_hal::{
    gpio::{
        gpioa::{PA5, PA6, PA7},
        gpiob::{PB0, PB1, PB10, PB11, PB2, PB6, PB7, PB8},
        Alternate, Input, OpenDrain, Output, PullDown,
    },
    i2c::BlockingI2c,
    pac::{I2C1, SPI1},
    spi::{Spi, Spi1NoRemap},
};

pub type Mux =
    Mux74hc4067<PB10<Output>, PB1<Output>, PB0<Output>, PB2<Output>, PB11<Input<PullDown>>>;
pub type MaxTempReader =
    Max6675<Spi<SPI1, Spi1NoRemap, (PA5<Alternate>, PA6, PA7<Alternate>), u8>, PB8<Output>>;
#[cfg(feature = "agas")]
pub type Stepper = A4988<
    PB15<Output<PushPull>>,
    PB14<Output<PushPull>>,
    PB13<Output<PushPull>>,
    PB12<Output<PushPull>>,
    PB9<Output<PushPull>>,
>;
pub type Lcd = LcdI2c<BlockingI2c<I2C1, (PB6<Alternate<OpenDrain>>, PB7<Alternate<OpenDrain>>)>>;

/*
Ziegler-Nichols PID tuning
Kc --> Proportional gain
Pc --> Period of oscillations
|----------------------------------------|
| Control	| P      |	Ti      | Td     |
|-----------|--------|----------|--------|
| P	        | 0.5Kc  |	-       | -      |
| PI	    | 0.45Kc |	Pc/1.2  | -      |
| PID	    | 0.60Kc |	Pc/2    | Pc/8   |
|----------------------------------------|
 */

/* GAS STEPPER OPERATION
Max gas consumption calculation
1 kg/hr = 8.8 lpm

SCPV valve delivers at full open condition
- 6.25 kg/hr (55 lpm/2 cfm) @ 1.5 bar (20psi)
- 9.65 kg/hr (85 lpm/3 cfm) @ 2.0 bar (30psi)

https://www.clippard.com/downloads/PDF_Documents/Product%20Data%20Sheets/Clippard%20SCPV%20Proportional%20Valve.pdf
https://www.clippard.com/downloads/PDF_Documents/Product%20Data%20Sheets/Clippard%20SCPVD%20Proportional%20Valve%20Driver%20IO%20Manual.pdf

Flow Resolution 0.7 l/min per step
Position Resolution 0.001” per step ie 1000 steps for 1 revolution

| MS1 | MS2 | MS3 |    Step   | Excitation Mode |
|-----+-----+-----+-----------+-----------------|
|  0  |  0  |  0  | Full Step | 2 phase         |
|  1  |  0  |  0  | 1/2 Step  | 1-2 phase       |
|  0  |  1  |  0  | 1/4 Step  | W1-2 phase      |
|  1  |  1  |  0  | 1/8 Step  | 2W1-2 phase     |
|  1  |  1  |  1  | 1/16 Step | 4W1-2 phase     |
|-----+-----+-----+-----------+-----------------|

For full flow of 55.0lpm @ 1.5bar 400 steps is needed
Each step input (high/low) requires a 1us delay
|------------------------------------------------------------------------------|
|	Flow %	|	Steps(1) |	Steps(1/2) | Steps(1/4) | Steps(1/8) | Steps(1/16) |
|-----------|------------|-------------|------------|------------|-------------|
|	0	    |	0	     |	0	       | 0          | 0	         | 0	       |
|	10	    |	40	     |	80	       | 160	    | 320        | 640	       |
|	20	    |	80	     |	160	       | 320	    | 640        | 1280	       |
|	30	    |	120	     |	240	       | 480	    | 960        | 1920	       |
|	40	    |	160	     |	320	       | 640	    | 1280	     | 2560	       |
|	50	    |	200	     |	400	       | 800	    | 1600	     | 3200	       |
|	60	    |	240	     |	480	       | 960	    | 1920	     | 3840	       |
|	70	    |	280	     |	560	       | 1120	    | 2240	     | 4480	       |
|	80	    |	320	     |	640	       | 1280	    | 2560	     | 5120	       |
|	90	    |	360	     |	720	       | 1440	    | 2880	     | 5760	       |
|	100	    |	400	     |	800	       | 1600	    | 3200	     | 6400	       |
|------------------------------------------------------------------------------|
*/
