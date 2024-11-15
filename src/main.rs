#![no_std]
#![no_main]


/**** low-level imports *****/

use core::{fmt::Write, panic::PanicInfo};
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;

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

/***** peripheral imports *****/

// USB Device support
use usb_device::class_prelude::*;
// USB Communications Class Device support
mod usb_manager;
use usb_manager::UsbManager;
// USB managment objects
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut USB_MANAGER: Option<UsbManager> = None;
// NeoMatrix crates
use ws2812_pio::Ws2812;
use smart_leds::{RGB8, SmartLedsWrite};
// LIS3DH support
use lis3dh::{Lis3dh, SlaveAddr};
use accelerometer::{Accelerometer, vector::F32x3};
// const WHO_AM_I       : u8 = 0x0F; // lis3dh WHO AM I register address
// animations.rs imports
mod animations;
use animations::{Bouncy, Bright, Snake, Spiral};

// USB Insurance
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

#[entry]
fn main() -> ! {
    let pac: pac::Peripherals = pac::Peripherals::take().unwrap();
    let mut resets: pac::RESETS = pac.RESETS;

    // initialize the watchdog timer, to pass into the clock init
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

    // initialize the timer
    let timer: Timer = Timer::new(pac.TIMER, &mut resets, &clocks);
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut resets);
    let mut neopixels: Ws2812<pac::PIO0, hal::pio::SM0, hal::timer::CountDown<'_>, hal::gpio::Pin<hal::gpio::bank0::Gpio7, hal::gpio::FunctionPio0, hal::gpio::PullDown>> = Ws2812::new(
        pins.d5.into_function(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    // initialize the delay timer
    let core = pac::CorePeripherals::take().unwrap();
    let mut delay_timer: cortex_m::delay::Delay = cortex_m::delay::Delay::new(
        core.SYST, 
        clocks.system_clock.freq().to_Hz()
    );

    // enable the propmaker power pin to power NeoMatrix
    let mut pwr_pin: hal::gpio::Pin<hal::gpio::bank0::Gpio10, hal::gpio::FunctionSio<hal::gpio::SioOutput>, hal::gpio::PullDown> = pins.d10.into_push_pull_output();
    pwr_pin.set_high().unwrap();

    // initialize the usb manager
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

    // initialize I2C to read from the LIS3DH
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

    // initialize the accelerometer
    let mut lis3dh = Lis3dh::new_i2c(i2c, SlaveAddr::Default).unwrap();
    lis3dh.set_range(lis3dh::Range::G2).unwrap();
    lis3dh.set_datarate(lis3dh::DataRate::Hz_400).unwrap();
    
    // initialize animation structs
    let mut bright: Bright   = Bright::new(RGB8::new(0,20,30));
    let mut snake: Snake = Snake::new(RGB8::new(30, 0 ,10));
    let mut spiral: Spiral = Spiral::new(RGB8::new(20,0,20));
    let mut bouncy: Bouncy = Bouncy::new(RGB8::new(10, 30, 0));

    // initialize accelerometer value variables
    let mut x_accel: f32;
    let mut y_accel: f32;
    let mut z_accel: f32;

    // mode control variable
    let mut mode : u8 = 1;

    // tick interval (5 ms)
    let mut nticks : u8 = 5;

    loop {

        // read the accel data
        let accel_data: F32x3 = lis3dh.accel_norm().unwrap();
        x_accel = accel_data.x;
        y_accel = accel_data.y;
        z_accel = accel_data.z;

        // write to the terminal
        write!(usb_bus, "X: {:?}\r\n", x_accel).unwrap();
        write!(usb_bus, "Y: {:?}\r\n", y_accel).unwrap();
        write!(usb_bus, "Z: {:?}\r\n\n", z_accel).unwrap();

        // choose mode based off direction
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

        // ensure 25 ms have passed
        if nticks > 4 {

            // to terminal
            write!(usb_bus, "updating display..\r\n").unwrap();

            //reset counter
            nticks = 0;

            // iterate thru the applicable nodes
            bright.next();
            snake.next();
            spiral.next();
            bouncy.next();

            // select list based off current mode
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

        // count!
        nticks += 1;
        // delay 5ms
        delay_timer.delay_ms(5 as u32);
    }

}
