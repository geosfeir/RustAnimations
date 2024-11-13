#![no_std]
#![no_main]

use core::{fmt::Write, panic::PanicInfo};

/**** low-level imports *****/
// use panic_halt as _;
// use cortex_m::peripheral::Peripherals;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
// use embedded_time::rate::*;

/***** board-specific imports *****/
use adafruit_feather_rp2040::hal::{self as hal};
use adafruit_feather_rp2040::{
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        watchdog::Watchdog,
        Sio,
        i2c::I2C,
        pac,
        pac::interrupt,
        pio::PIOExt,
        Timer,
        fugit::RateExtU32,
    },
    Pins, XOSC_CRYSTAL_FREQ,
};

// use usbd_serial::SerialPort;
// USB Device support
use usb_device::class_prelude::*;
// use usb_device::prelude::UsbDeviceBuilder;
// USB Communications Class Device support
mod usb_manager;
use usb_manager::UsbManager;

static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut USB_MANAGER: Option<UsbManager> = None;

#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ(){
    match USB_MANAGER.as_mut() {
        Some(manager) => manager.interrupt(),
        None => (),
    };
}
#[panic_handler]
fn panic(panic_info: &PanicInfo) -> ! {
    if let Some(usb) = unsafe { USB_MANAGER.as_mut() } {
        writeln!(usb, "{}", panic_info).ok();
    }
    loop {}
}

mod animations;
use animations::{Bouncy, Bright, Snake, Spiral};
use ws2812_pio::Ws2812;
use smart_leds::{RGB8, SmartLedsWrite};
use lis3dh::{Lis3dh, SlaveAddr};
use accelerometer::{Accelerometer, vector::F32x3};
// const WHO_AM_I       : u8 = 0x0F;

#[entry]
fn main() -> ! {
    // Grab the singleton objects
    let pac: pac::Peripherals = pac::Peripherals::take().unwrap();
    let mut resets: pac::RESETS = pac.RESETS;
    let core = pac::CorePeripherals::take().unwrap();
    // Init the watchdog timer, to pass into the clock init
    let mut watchdog: Watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks: hal::clocks::ClocksManager = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut resets,
        &mut watchdog,
    ).ok().unwrap();

    // initialize the Single Cycle IO
    let sio = Sio::new(pac.SIO);
    // initialize the pins to default state
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut resets,
    );

    let mut delay_timer: cortex_m::delay::Delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    // propmaker power pin
    let mut pwr_pin: hal::gpio::Pin<hal::gpio::bank0::Gpio10, hal::gpio::FunctionSio<hal::gpio::SioOutput>, hal::gpio::PullDown> = pins.d10.into_push_pull_output();
    pwr_pin.set_high().unwrap();


    let timer: Timer = Timer::new(pac.TIMER, &mut resets, &clocks);
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut resets);
    let mut neopixels: Ws2812<pac::PIO0, hal::pio::SM0, hal::timer::CountDown<'_>, hal::gpio::Pin<hal::gpio::bank0::Gpio7, hal::gpio::FunctionPio0, hal::gpio::PullDown>> = Ws2812::new(
        pins.d5.into_function(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let mut bright: Bright   = Bright::new(RGB8::new(0,20,30));
    let mut snake: Snake = Snake::new(RGB8::new(30, 0 ,10));
    let mut spiral: Spiral = Spiral::new(RGB8::new(20,0,20));
    let mut bouncy: Bouncy = Bouncy::new(RGB8::new(10, 30, 0));

    let usb_bus: &mut UsbManager = unsafe {
        USB_BUS = Some(UsbBusAllocator::new(hal::usb::UsbBus::new(
            pac.USBCTRL_REGS,
            pac.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        )));
        USB_MANAGER = Some(UsbManager::new(USB_BUS.as_ref().unwrap()));
        // Enable the USB interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
        USB_MANAGER.as_mut().unwrap()
    };

    // let mut serial = SerialPort::new(usb_bus);

    // // create USB device with VID and PID
    // let mut usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd)).device_class(2).build();
    
    let mut x_accel: f32;
    let mut y_accel: f32;
    let mut z_accel: f32;
    
    let freq: fugit::Rate<u32, 1, 1> = 400.kHz();
    let sda_pin: hal::gpio::Pin<hal::gpio::bank0::Gpio2, hal::gpio::FunctionI2c, hal::gpio::PullDown> = pins.sda.into_function::<hal::gpio::FunctionI2C>();
    let scl_pin: hal::gpio::Pin<hal::gpio::bank0::Gpio3, hal::gpio::FunctionI2c, hal::gpio::PullDown> = pins.scl.into_function::<hal::gpio::FunctionI2C>();

    let i2c: I2C<pac::I2C1, (hal::gpio::Pin<hal::gpio::bank0::Gpio2, hal::gpio::FunctionI2c, hal::gpio::PullDown>, hal::gpio::Pin<hal::gpio::bank0::Gpio3, hal::gpio::FunctionI2c, hal::gpio::PullDown>)> = I2C::i2c1(
        pac.I2C1,
        sda_pin, 
        scl_pin, 
        freq, 
        &mut resets, 
        &clocks.system_clock
    );

    let mut lis3dh: Lis3dh<lis3dh::Lis3dhI2C<I2C<pac::I2C1, (hal::gpio::Pin<hal::gpio::bank0::Gpio2, hal::gpio::FunctionI2c, hal::gpio::PullDown>, hal::gpio::Pin<hal::gpio::bank0::Gpio3, hal::gpio::FunctionI2c, hal::gpio::PullDown>)>>> = Lis3dh::new_i2c(i2c, SlaveAddr::Default).unwrap();
    lis3dh.set_range(lis3dh::Range::G2).unwrap();
    lis3dh.set_datarate(lis3dh::DataRate::Hz_400).unwrap();

    /*
    Loop Section
    */
    let mut mode : u8 = 1;   // loop delay in ms
    let mut nticks : u8 = 5;

    loop {
        // // check for USB device
        // if usb_dev.poll(&mut [&mut serial]) {
        //     let mut buf = [0u8; 64];
        //     match serial.read(&mut buf) {
        //         Err(_e) => {
        //             // nada
        //         }
        //         Ok(0) => {
        //             // nada
        //         }
        //         Ok(count) => {
        //             // convert to upper case
        //             buf.iter_mut().take(count).for_each(|b: &mut u8| {
        //                 b.make_ascii_uppercase();
        //             });
        //             // send back to host
        //             let mut wr_ptr: &[u8] = &buf[..count];
        //             while !wr_ptr.is_empty() {
        //                 match serial.write(wr_ptr) {
        //                     Ok(len) => wr_ptr = &wr_ptr[len..],
        //                     // on error, drop unwritten data
        //                     // one possible error is the buffer being full
        //                     Err(_) =>  break,
        //                 };
        //             }
        //         }
        //     }
        // }

        let accel_data: F32x3 = lis3dh.accel_norm().unwrap();

        x_accel = accel_data.x;
        y_accel = accel_data.y;
        z_accel = accel_data.z;

        write!(usb_bus, "X: {:?}\r\n", x_accel).unwrap();
        write!(usb_bus, "Y: {:?}\r\n", y_accel).unwrap();
        write!(usb_bus, "Z: {:?}\r\n\n", z_accel).unwrap();

        if x_accel > 0.1 {
            mode = 0;
        } else if x_accel < -0.1 {
            mode = 1;
        }

        if y_accel > 0.1 {
            mode = 2;
        } else if y_accel < -0.1 {
            mode = 3;
        }

        if nticks > 4 {
            write!(usb_bus, "updating display..\r\n").unwrap();
            nticks = 0;
            // itr thru the applicable nodes
            bright.next();
            snake.next();
            spiral.next();
            bouncy.next();

            // select list based off current node
            let ds: [RGB8; animations::NUM_PX] = match mode {
                0 => bright.to_list(),
                1 => snake.to_list(),
                2 => spiral.to_list(),
                3 => bouncy.to_list(),
                _ => [RGB8::new(0,0,0); animations::NUM_PX],
            };

            // write to NeoMatrix!!!
            neopixels.write(ds.iter().cloned()).unwrap();
        }

        nticks += 1;
        delay_timer.delay_ms(5 as u32);
    }

}
