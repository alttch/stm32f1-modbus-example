#![no_std]
#![no_main]

#[macro_use]
extern crate fixedvec;
use core::ptr;
use core::sync::atomic::{self, Ordering};

use fixedvec::FixedVec;

use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
#[allow(unused_imports)]
use panic_halt;
use stm32f1xx_hal::{
    delay::Delay,
    pac,
    prelude::*,
    serial::{Config, Serial},
};

use rmodbus::server::{context, guess_frame_len, process_frame, ModbusFrame, ModbusProto};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh); // will blink on frame rcv
    let mut flash = dp.FLASH.constrain();
    let clocks = rcc.cfgr.sysclk(14.mhz()).freeze(&mut flash.acr);
    let mut delay = Delay::new(cp.SYST, clocks);

    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let tx = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
    let rx = gpiob.pb11;
    let mut direction = gpiob.pb12.into_push_pull_output(&mut gpiob.crh); // dir ctrl 4 half duplex
    let serial = Serial::usart3(
        // B10=DI, B11=R0, B12=DE+RE
        dp.USART3,
        (tx, rx),
        &mut afio.mapr,
        Config::default()
            .baudrate(9600.bps())
            .parity_none()
            .stopbits(stm32f1xx_hal::serial::StopBits::STOP1),
        clocks,
        &mut rcc.apb1,
    );
    let (mut _tx, mut _rx) = serial.split();
    let channels = dp.DMA1.split(&mut rcc.ahb);
    let mut rx = _rx.with_dma(channels.3);
    let mut tx = _tx.with_dma(channels.2);
    direction.set_low().ok();
    // pre-setup DMA channels
    // working with DMA directly because stm32f1xx-hal read/write with DMA are limited to args
    rx.channel.ch().cr.modify(|_, w| {
        w.mem2mem()
            .clear_bit()
            .pl()
            .medium()
            .msize()
            .bits8()
            .psize()
            .bits8()
            .circ()
            .clear_bit()
            .dir()
            .clear_bit()
    });
    rx.channel.set_peripheral_address(
        unsafe { &(*pac::USART3::ptr()).dr as *const _ as u32 },
        false,
    );
    tx.channel.ch().cr.modify(|_, w| {
        w.mem2mem()
            .clear_bit()
            .pl()
            .medium()
            .msize()
            .bits8()
            .psize()
            .bits8()
            .circ()
            .clear_bit()
            .dir()
            .set_bit()
    });
    tx.channel.set_peripheral_address(
        unsafe { &(*pac::USART3::ptr()).dr as *const _ as u32 },
        false,
    );
    let mut c: u32 = 0; // frame calc
    loop {
        macro_rules! exec_channel {
            ($ch:expr, $obj:expr, $len:expr, $nto:expr) => {
                $ch.set_memory_address($obj.as_ptr() as u32, true);
                $ch.set_transfer_length($len);
                atomic::compiler_fence(Ordering::Release);
                $ch.start();
                let mut to = 0;
                let timeout = $len * 2;
                while $ch.in_progress() {
                    delay.delay_ms(1_u16);
                    if $nto {
                        let remaining = $ch.get_ndtr() as usize;
                        if remaining < $len {
                            to = to + 1;
                            if to > timeout {
                                break;
                            }
                        }
                    }
                }
                atomic::compiler_fence(Ordering::Acquire);
                $ch.stop();
                unsafe {
                    ptr::read_volatile(&0);
                }
                atomic::compiler_fence(Ordering::Acquire);
            };
        }
        led.set_high().unwrap();
        let frame: ModbusFrame = [0; 256];
        exec_channel!(rx.channel, frame, 8, true);
        let len = guess_frame_len(&frame, ModbusProto::Rtu).unwrap();
        if len > 8 {
            exec_channel!(rx.channel, frame[8..], (len - 8) as usize, true);
        }
        led.set_low().unwrap();
        c = c + 1;
        if c == u32::MAX {
            c = 0;
        }
        context::input_set_u32(0, c).unwrap(); // input registers 0-1 = frame calc (u32 big endian)
        let mut preallocated_space = alloc_stack!([u8; 256]);
        let mut response: FixedVec<u8> = FixedVec::new(&mut preallocated_space);
        if process_frame(1, &frame, ModbusProto::Rtu, &mut response).is_ok() && !response.is_empty()
        {
            direction.set_high().ok();
            exec_channel!(tx.channel, response.as_slice(), response.len(), false);
            delay.delay_ms(response.len() as u16); // make sure frame is sent before switching dir
            direction.set_low().ok();
        }
    }
}
