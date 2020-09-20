#![no_main]
#![no_std]

use aux14::i2c1;
use aux14::{entry, Direction, iprintln};
use futures::stream::StreamExt;
use futures::{stream, Stream};

use core::f32::consts::PI;
use inefficient::BoolFuture;
// this trait provides the `atan2` method
use f3::hal::stm32f30x::{rcc, tim6, RCC, TIM6, ITM};
use futures::future::Either;
use m::Float;

// Slave address
const MAGNETOMETER: u8 = 0b001_1110;

// Addresses of the magnetometer's register that has the magnetic data
const OUT_X_H_M: u8 = 0x03;

/// Inefficient but (I think) valid implementations of handy async functions in Rust
mod inefficient {
    use core::future::Future;
    use core::pin::Pin;
    use core::task::{Context, Poll};

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

fn get_compass_forever(i2c1: &'static i2c1::RegisterBlock) -> impl Stream<Item = (i16, i16, i16)> {
    stream::repeat(()).then(move |()| get_compass(i2c1))
}

pub fn init_timer() -> &'static tim6::RegisterBlock {
    let rcc: &'static rcc::RegisterBlock = unsafe { &*RCC::ptr() };

    // Power on the TIM6 timer
    rcc.apb1enr.modify(|_, w| w.tim6en().set_bit());

    let tim6: &'static tim6::RegisterBlock = unsafe { &*TIM6::ptr() };

    // OPM Select one pulse mode
    // CEN Keep the counter disabled for now
    tim6.cr1.write(|w| w.opm().set_bit().cen().clear_bit());

    // Configure the prescaler to have the counter operate at 1 KHz
    // APB1_CLOCK = 8 MHz
    // PSC = 7999
    // 8 MHz / (7999 + 1) = 1 KHz
    // The counter (CNT) will increase on every millisecond
    tim6.psc.write(|w| w.psc().bits(7_999));

    tim6
}

pub async fn delay(ms: u16, tim6: &'static tim6::RegisterBlock) {
    // set timer to go off in `ms` milliseconds
    tim6.arr.write(|w| w.arr().bits(ms));

    // CEN: enable the counter
    tim6.cr1.modify(|_, w| w.cen().set_bit());

    BoolFuture(|| tim6.sr.read().uif().bit_is_set()).await;

    // clear the update event flag
    tim6.sr.modify(|_, w| w.uif().clear_bit());
}

fn delay_forever(ms: u16, tim6: &'static tim6::RegisterBlock) -> impl Stream<Item = ()> {
    stream::repeat(()).then(move |()| delay(ms, tim6))
}

fn mag_to_angle(mag: (i16, i16, i16)) -> f32 {
    let (x, y, _z) = mag;

    (y as f32).atan2(x as f32) / PI * 180.0 // in degrees
}

// Angle in degrees
fn angle_to_direction(angle: f32) -> Direction {
    let angle_chunked = (angle + 22.5) / 45.0;
    let angle_rounded = angle_chunked as u32;
    match angle_rounded {
        0 => Direction::South,
        1 => Direction::Southeast,
        2 => Direction::East,
        3 => Direction::Northeast,
        4 => Direction::North,
        5 => Direction::Northwest,
        6 => Direction::West,
        7 => Direction::Southwest,
        8 => Direction::South,
        _ => panic!("Illegal angle: {}", angle)
    }
}

const TIMER_MS: u16 = 100;
const TIMER_S: f32 = 0.1;

#[entry]
fn main() -> ! {
    let (mut leds, i2c1, _delay, mut itm) = aux14::init();
    let timer = init_timer();

    use rand::{RngCore, SeedableRng};
    use rand::rngs::SmallRng;

    // Use data from the compass to seed the RNG
    let (x, y, z) = spin_on::spin_on(get_compass(i2c1));
    let mut seed = 0u64;
    seed += u64::from(u16::from_be_bytes(x.to_be_bytes()));
    seed += u64::from(u16::from_be_bytes(y.to_be_bytes())) << 16;
    seed += u64::from(u16::from_be_bytes(z.to_be_bytes())) << 32;
    let mut rng = SmallRng::seed_from_u64(seed);
    let x = rng.next_u32();
    iprintln!(&mut itm.stim[0], "{:?}", x);

    let mut position_xy_m = (0.0, 0.0);
    let mut timer_cycle = 0usize;
    let mut last_mag = (0, 0, 0);
    spin_on::spin_on(
        stream::select(
            get_compass_forever(i2c1).map(Either::Left),
            delay_forever(TIMER_MS, timer).map(Either::Right),
        )
        .for_each(|either| {
            match either {
                Either::Left(mag) => {
                    last_mag = mag;
                }
                Either::Right(()) => {
                    timer_cycle = (timer_cycle + 1) % 2;
                    let (x, y, _z) = last_mag;
                    let x = f32::from(x);
                    let y = f32::from(y);
                    let norm = (x * x + y * y).sqrt();
                    if norm < f32::EPSILON {
                        let x = x / norm;
                        let y = y / norm;
                        position_xy_m = (
                            position_xy_m.0 + x * TIMER_S,
                            position_xy_m.1 + y * TIMER_S,
                        );
                        // iprintln!(&mut itm.stim[0], "{:?}", position_xy_m)
                    }
                }
            }

            let angle = (mag_to_angle(last_mag) + 360.0) % 360.0;
            let mag_dir = angle_to_direction(angle);

            leds.iter_mut().for_each(|led| led.off());
            leds[mag_dir].on();

            futures::future::ready(())
        }),
    );
    unreachable!("Because the stream is infinite")
}
