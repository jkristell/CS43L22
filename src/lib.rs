#![no_std]

use embedded_hal::{
    blocking::{delay::DelayMs, i2c},
    digital::v2::OutputPin,
};

// AD0 is connected to ground so devaddr is 0x94
const DEV_I2C_ADDR: u8 = 0x94 >> 1;

pub enum CodecRegs {
    Id = 0x01,
    PowerCtl1 = 0x02,
    // 0x03 - Reserved
    PowerCtl2 = 0x04,
    ClockingCtl = 0x05,
    InterfaceCtl1 = 0x06,
    InterfaceCtl2 = 0x07,
    PassthroughASelect = 0x08,
    PassthroughBSelect = 0x09,
    AnalogZcSrSettings = 0x0A,
    PassthroughGangControl = 0x0C,
    PlaybackCtl1 = 0x0D,
    Misc = 0x0E,
    PlaybackCtl2 = 0x0F,
}

impl Into<u8> for CodecRegs {
    fn into(self) -> u8 {
        self as u8
    }
}


pub struct Cs43l22<I2C, RESETPIN> {
    /// i2c device
    i2c: I2C,
    /// reset pin
    reset_pin: RESETPIN,
}

impl<I2C, I2CERR, RESETPIN, PINERR> Cs43l22<I2C, RESETPIN>
where
    I2C: i2c::WriteRead<Error = I2CERR> + i2c::Write<Error = I2CERR>,
    RESETPIN: OutputPin<Error = PINERR>,
{
    pub fn new(i2c: I2C, reset_pin: RESETPIN) -> Self {
        Cs43l22 { i2c, reset_pin }
    }

    pub fn codec_reset<DELAY: DelayMs<u8>>(&mut self, delay: &mut DELAY) {
        //TODO: Pin Error handling

        let _ = self.reset_pin.set_low();

        //TODO: Check real delay time
        delay.delay_ms(100);

        let _ = self.reset_pin.set_high();
        delay.delay_ms(100);
    }

    pub fn codec_init<DELAY: DelayMs<u8>>(&mut self, delay: &mut DELAY, vol: u8) {
        use CodecRegs::*;

        self.codec_reset(delay);

        /* Keep Codec powered OFF */
        self.write_reg(PowerCtl1, 0x01);

        /* SPK always OFF & HP always ON */
        self.write_reg(PowerCtl2, 0xAF);

        /* Clock configuration: Auto detection */
        self.write_reg(ClockingCtl, 0x81);

        /* Set the Slave Mode and the audio Standard */
        // Philips
        self.write_reg(InterfaceCtl1, 0x04);

        /* Set the Master volume */
        self.codec_setvol(vol);

        //NOTE: Skipped a block here, we use i2s

        /* Power on the Codec */
        self.write_reg(PowerCtl1, 0x9E);

        /* Additional configuration for the CODEC. These configurations are done to reduce
        the time needed for the Codec to power off. If these configurations are removed,
        then a long delay should be added between powering off the Codec and switching
        off the I2S peripheral MCLK clock (which is the operating clock for Codec).
        If this delay is not inserted, then the codec will not shut down properly and
        it results in high noise after shut down. */

        /* Disable the analog soft ramp */
        self.write_reg(0x0A, 0x00);

        /* Disable the digital soft ramp */
        self.write_reg(0x0E, 0x04);
        /* Disable the limiter attack level */
        self.write_reg(0x27, 0x00);
        /* Adjust Bass and Treble levels */
        self.write_reg(0x1F, 0x0F);
        /* Adjust PCM volume level */
        self.write_reg(0x1A, 0x0A);
        self.write_reg(0x1B, 0x0A);

        /* Configure the I2S peripheral */
        // JK: This should be done by the Hal. It's just the setup of the I2S peripheral
        //     But the settings needs to be known by the codec.
        //TODO: Codec_AudioInterface_Init(AudioFreq);
    }

    pub fn codec_audiointerface_init(&mut self) {

    }

    pub fn codec_setvol(&mut self, vol: u8) {
        //TODO: Check how vol register works

        if (vol > 0xE6) {
            /* Set the Master volume */
            self.write_reg(0x20, vol - 0xE7);
            self.write_reg(0x21, vol - 0xE7);
        } else {
            /* Set the Master volume */
            self.write_reg(0x20, vol + 0x19);
            self.write_reg(0x21, vol + 0x19);
        }
    }

    /// Write register
    pub fn write_reg<T: Into<u8>>(&mut self, addr: T, value: u8) -> Result<(), I2CERR> {
        self.i2c.write(DEV_I2C_ADDR, &[addr.into(), value])
    }

    /// Read register
    pub fn read_reg<T: Into<u8>>(&mut self, addr: T) -> Result<u8, I2CERR> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(DEV_I2C_ADDR, &[addr.into()], &mut buf)?;
        Ok(buf[0])
    }
}

/*
static uint32_t Codec_Init(uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq)
{
  uint32_t counter = 0;

  /* Configure the Codec related IOs */
  Codec_GPIO_Init();

  /* Reset the Codec Registers */
  Codec_Reset();

  /* Initialize the Control interface of the Audio Codec */
  Codec_CtrlInterface_Init();

  /* Keep Codec powered OFF */
  counter += Codec_WriteRegister(0x02, 0x01);

  counter += Codec_WriteRegister(0x04, 0xAF); /* SPK always OFF & HP always ON */
  OutputDev = 0xAF;

  /* Clock configuration: Auto detection */
  counter += Codec_WriteRegister(0x05, 0x81);

  /* Set the Slave Mode and the audio Standard */
  counter += Codec_WriteRegister(0x06, CODEC_STANDARD);

  /* Set the Master volume */
  Codec_VolumeCtrl(Volume);

  if (CurrAudioInterface == AUDIO_INTERFACE_DAC)
  {
    /* Enable the PassThrough on AIN1A and AIN1B */
    counter += Codec_WriteRegister(0x08, 0x01);
    counter += Codec_WriteRegister(0x09, 0x01);

    /* Route the analog input to the HP line */
    counter += Codec_WriteRegister(0x0E, 0xC0);

    /* Set the Passthough volume */
    counter += Codec_WriteRegister(0x14, 0x00);
    counter += Codec_WriteRegister(0x15, 0x00);
  }

  /* Power on the Codec */
  counter += Codec_WriteRegister(0x02, 0x9E);

  /* Additional configuration for the CODEC. These configurations are done to reduce
      the time needed for the Codec to power off. If these configurations are removed,
      then a long delay should be added between powering off the Codec and switching
      off the I2S peripheral MCLK clock (which is the operating clock for Codec).
      If this delay is not inserted, then the codec will not shut down properly and
      it results in high noise after shut down. */

  /* Disable the analog soft ramp */
  counter += Codec_WriteRegister(0x0A, 0x00);
  if (CurrAudioInterface != AUDIO_INTERFACE_DAC)
  {
    /* Disable the digital soft ramp */
    counter += Codec_WriteRegister(0x0E, 0x04);
  }
  /* Disable the limiter attack level */
  counter += Codec_WriteRegister(0x27, 0x00);
  /* Adjust Bass and Treble levels */
  counter += Codec_WriteRegister(0x1F, 0x0F);
  /* Adjust PCM volume level */
  counter += Codec_WriteRegister(0x1A, 0x0A);
  counter += Codec_WriteRegister(0x1B, 0x0A);

  /* Configure the I2S peripheral */
  Codec_AudioInterface_Init(AudioFreq);

  /* Return communication control value */
  return counter;
}
*/
