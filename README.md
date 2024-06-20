# AudioDSP-v3.1.1
## Description
Audio DSP - 3.1.1
### Board Code
- 3: Based on AudioDSP-v3.0 hardware
- 1: Audio Input Interface
   - 5.1 Analog RCA
- 1: Audio Output Interface
   - 5.1 Analog RCA

## Features
- DC Input +6V/1A
- Audio Channels: 5.1
- SigmaDSP ADAU1452
   - I2C control for DSP
- Sampling frequency (default): 192KHz
- 24 bit ADC PCM1862 x3
- 24 bit DAC PCM1681 x1
- ST uC STM32F401RxT6 with Serial Wire programming
- Synchronous Clock Distribution
- LED indicators:
   - +3V3
   - +5V0
   - DSP