#![no_main]
#![no_std]

use aux14::i2c1;
use aux14::{entry, iprintln, Direction};
use futures::stream;
use futures::stream::StreamExt;

use core::f32::consts::PI;
use inefficient::BoolFuture;
// this trait provides the `atan2` method
use m::Float;

// Slave address
const MAGNETOMETER: u8 = 0b001_1110;

// Addresses of the magnetometer's register that has the magnetic data
const OUT_X_H_M: u8 = 0x03;

/// Inefficient but (I think) valid implementations of handy async functions in Rust
mod inefficient {
    use core::future::Future;
    use core::pin::Pin;
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

    unsafe fn rwclone(_p: *const ()) -> RawWaker {
        noop_waker()
    }

    unsafe fn rwwake(_p: *const ()) {}

    unsafe fn rwwakebyref(_p: *const ()) {}

    unsafe fn rwdrop(_p: *const ()) {}

    static VTABLE: RawWakerVTable = RawWakerVTable::new(rwclone, rwwake, rwwakebyref, rwdrop);

    /// The simplest way to create a noop waker in Rust. You would only ever want to use this with
    /// an executor that polls continuously. I got this implementation from user 2e71828 on
    /// [this Rust forum post](https://users.rust-lang.org/t/simplest-possible-block-on/48364/2).
    fn noop_waker() -> RawWaker {
        static DATA: () = ();
        RawWaker::new(&DATA, &VTABLE)
    }

    /// Continuously poll a future until it returns `Poll::Ready`. This is not normally how an
    /// executor should work, because it runs the CPU at 100%.
    pub fn block_on<F: Future>(future: F) -> F::Output {
        pin_utils::pin_mut!(future);
        let waker = &unsafe { Waker::from_raw(noop_waker()) };
        let mut cx = Context::from_waker(waker);
        loop {
            if let Poll::Ready(output) = future.as_mut().poll(&mut cx) {
                return output;
            }
        }
    }

    /// Convert a function that returns bool into a valid but very inefficient future.
    /// This will return `Poll::Ready` if and only if the function returns true.
    /// The key trick
    /// to make this valid is that we always call the waker if we are going to return `Pending`.
    /// That way the executor is guaranteed to continue polling us. This doesn't actually matter if
    /// we're using the `block_on` executor from this mod, but it would matter if we used a normal
    /// executor. I got this trick from user HadrienG in [this Rust forum post](https://users.rust-lang.org/t/polling-in-new-era-futures/30531/2).
    pub struct BoolFuture<F: Fn() -> bool>(pub F);

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
}

/// It's only legal to call this function once at a time, i.e. you can't call get_compass while
/// another copy of get_compass is running. Also, in order to leave the I2C bus in a valid state,
/// you must run this function to completion.
async fn get_compass(i2c1: &'static i2c1::RegisterBlock) -> (i16, i16, i16) {
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
        BoolFuture(|| i2c1.isr.read().rxne().bit_is_set()).await;

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

    (x, y, z)
}

#[entry]
fn main() -> ! {
    let (mut leds, i2c1, _delay, mut itm) = aux14::init();

    inefficient::block_on(
        stream::repeat(())
            .then(|()| get_compass(i2c1))
            .map(|mag| {
                iprintln!(&mut itm.stim[0], "{:?}", mag);

                let (x, y, _z) = mag;

                let theta = (y as f32).atan2(x as f32); // in radians

                let dir = if theta < -7. * PI / 8. {
                    Direction::North
                } else if theta < -5. * PI / 8. {
                    Direction::Northwest
                } else if theta < -3. * PI / 8. {
                    Direction::West
                } else if theta < -PI / 8. {
                    Direction::Southwest
                } else if theta < PI / 8. {
                    Direction::South
                } else if theta < 3. * PI / 8. {
                    Direction::Southeast
                } else if theta < 5. * PI / 8. {
                    Direction::East
                } else if theta < 7. * PI / 8. {
                    Direction::Northeast
                } else {
                    Direction::North
                };

                leds.iter_mut().for_each(|led| led.off());
                leds[dir].on();
            })
            .for_each(|()| futures::future::ready(())),
    );
    unreachable!("Because the stream is infinite")
}
