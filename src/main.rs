#![deny(unsafe_code)]
#![no_main]
#![no_std]

#[allow(unused_imports)]
use aux14::{entry, iprint, iprintln, prelude::*};

// Slave address
const MAGNETOMETER: u8 = 0b001_1110;

// Addresses of the magnetometer's registers
const OUT_X_H_M: u8 = 0x03;
const IRA_REG_M: u8 = 0x0A;

#[entry]
fn main() -> ! {
    let (i2c1, mut delay, mut itm) = aux14::init();

    loop {
        // Broadcast START
        // Broadcast the MAGNETOMETER address with the R/W bit set to Write
        i2c1.cr2.write(|w| {
            w.start().set_bit();
            w.sadd1().bits(MAGNETOMETER);
            w.rd_wrn().clear_bit();
            w.nbytes().bits(1);
            w.autoend().clear_bit()
        });

        // Wait until we can send more data
        while i2c1.isr.read().txis().bit_is_clear() {}

        // Send the address of the register that we want to read: OUT_X_H_M
        i2c1.txdr.write(|w| w.txdata().bits(OUT_X_H_M));

        // Wait until the previous byte has been transmitted
        while i2c1.isr.read().tc().bit_is_clear() {}

        // Broadcast RESTART
        // Broadcast the MAGNETOMETER address with the R/W bit set to Read
        i2c1.cr2.modify(|_, w| {
            w.start().set_bit();
            w.nbytes().bits(6);
            w.rd_wrn().set_bit();
            w.autoend().set_bit()
        });

        let mut buffer = [0u8; 6];
        for byte in &mut buffer {
            // Wait until we have received something
            while i2c1.isr.read().rxne().bit_is_clear() {}

            *byte = i2c1.rxdr.read().rxdata().bits();
        }
        // Broadcast STOP (automatic because of `AUTOEND = 1`)

        iprintln!(&mut itm.stim[0], "{:?}", buffer);

        let x_h = u16::from(buffer[0]);
        let x_l = u16::from(buffer[1]);
        let z_h = u16::from(buffer[2]);
        let z_l = u16::from(buffer[3]);
        let y_h = u16::from(buffer[4]);
        let y_l = u16::from(buffer[5]);

        let x = ((x_h << 8) + x_l) as i16;
        let y = ((y_h << 8) + y_l) as i16;
        let z = ((z_h << 8) + z_l) as i16;

        iprintln!(&mut itm.stim[0], "{:?}", (x, y, z));


        delay.delay_ms(1_000_u16);
    }
}
