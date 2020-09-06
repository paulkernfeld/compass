#![no_main]
#![no_std]

use aux14::i2c1;
#[allow(unused_imports)]
use aux14::{entry, iprint, iprintln, prelude::*};
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
use futures::future::FutureExt;
use futures::stream;
use futures::stream::StreamExt;
use futures::Stream;

// Slave address
const MAGNETOMETER: u8 = 0b001_1110;

// Addresses of the magnetometer's registers
const OUT_X_H_M: u8 = 0x03;
const IRA_REG_M: u8 = 0x0A;

// https://users.rust-lang.org/t/simplest-possible-block-on/48364/2
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

/// Wait until we can send more data
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

// Convert a function that returns bool into a valid but very inefficient future
// https://users.rust-lang.org/t/polling-in-new-era-futures/30531/2?u=occupy_paul_st
// Wait until the function returns true
struct BoolFuture<F: Fn() -> bool>(F);

impl<F: Fn() -> bool> Future for BoolFuture<F> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        if self.0() {
            Poll::Ready(())
        } else {
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    }
}

async fn get_compass(i2c1: &'static i2c1::RegisterBlock) {
// async fn get_compass(i2c1: &'static i2c1::RegisterBlock) -> (i16, i16, i16) {
    i2c1.cr2.write(|w| {
        w.start().set_bit();
        w.sadd1().bits(MAGNETOMETER);
        w.rd_wrn().clear_bit();
        w.nbytes().bits(1);
        w.autoend().clear_bit()
    });

    // Wait until we can send more data
    BoolFuture(|| i2c1.isr.read().txis().bit_is_set()).await;

    // Send the address of the register that we want to read: OUT_X_H_M
    i2c1.txdr.write(|w| w.txdata().bits(OUT_X_H_M));

    // Wait until the previous byte has been transmitted
    BoolFuture(|| i2c1.isr.read().tc().bit_is_set()).await;

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
        BoolFuture(|| i2c1.isr.read().rxne().bit_is_set());

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

    (x, y, z);
    ()
}

#[entry]
fn main() -> ! {
    let (i2c1, mut delay, mut itm) = aux14::init();

    block_on(stream::repeat(()).for_each(|()| {
        delay.delay_ms(1_000_u16);
        get_compass(i2c1)
        // get_compass(i2c1).map(|mag| {
        //     iprintln!(&mut itm.stim[0], "{:?}", mag);
        //     ()
        // })
    }));
    unreachable!("Because the stream is infinite")
}
