#![no_main]
#![no_std]

#[allow(unused_imports)]
use aux14::{entry, iprint, iprintln, prelude::*};
use core::future::Future;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
use futures::Stream;
use futures::stream::StreamExt;
use core::pin::Pin;
use aux14::i2c1;

// Slave address
const MAGNETOMETER: u8 = 0b001_1110;

// Addresses of the magnetometer's registers
const OUT_X_H_M: u8 = 0x03;
const IRA_REG_M: u8 = 0x0A;

// https://users.rust-lang.org/t/simplest-possible-block-on/48364/2?u=occupy_paul_st
unsafe fn rwclone(_p: *const ()) -> RawWaker {
    make_raw_waker()
}
unsafe fn rwwake(_p: *const ()) {}
unsafe fn rwwakebyref(_p: *const ()) {}
unsafe fn rwdrop(_p: *const ()) {}

static VTABLE: RawWakerVTable = RawWakerVTable::new(rwclone, rwwake, rwwakebyref, rwdrop);

fn make_raw_waker() -> RawWaker {
    static DATA: () = ();
    RawWaker::new(&DATA, &VTABLE)
}

fn block_on<F: Future>(future: F) -> F::Output {
    pin_utils::pin_mut!(future);
    let waker = &unsafe { Waker::from_raw(make_raw_waker()) };
    let mut cx = Context::from_waker(waker);
    loop {
        if let Poll::Ready(output) = future.as_mut().poll(&mut cx) {
            return output;
        }
    }
}

struct Txis(&'static i2c1::RegisterBlock);

impl Future for Txis {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        if self.0.isr.read().txis().bit_is_clear() {
            cx.waker().wake_by_ref();
            Poll::Pending
        } else {
            Poll::Ready(())
        }
    }
}

struct Compass(&'static i2c1::RegisterBlock);

impl Stream for Compass {
    type Item = (i16, i16, i16);

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        let i2c1 = &self.0;
        i2c1.cr2.write(|w| {
            w.start().set_bit();
            w.sadd1().bits(MAGNETOMETER);
            w.rd_wrn().clear_bit();
            w.nbytes().bits(1);
            w.autoend().clear_bit()
        });

        // Wait until we can send more data
        // TODO await
        block_on(Txis(i2c1));

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

        let x_h = u16::from(buffer[0]);
        let x_l = u16::from(buffer[1]);
        let z_h = u16::from(buffer[2]);
        let z_l = u16::from(buffer[3]);
        let y_h = u16::from(buffer[4]);
        let y_l = u16::from(buffer[5]);

        let x = ((x_h << 8) + x_l) as i16;
        let y = ((y_h << 8) + y_l) as i16;
        let z = ((z_h << 8) + z_l) as i16;

        Poll::Ready(Some((x, y, z)))
    }
}

#[entry]
fn main() -> ! {
    let (i2c1, mut delay, mut itm) = aux14::init();
    let mut compass = Compass(i2c1);

    loop {
        iprintln!(&mut itm.stim[0], "{:?}", block_on(compass.next()));

        delay.delay_ms(1_000_u16);
    }
}
