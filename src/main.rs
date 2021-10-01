#![no_std]
#![no_main]

use panic_halt as _;
use rtic::app;
use rtt_target::{rprintln, rtt_init_print};

use stm32_usbd::UsbBus;
use stm32_usbd::UsbPeripheral;
use stm32l4xx_hal::device::RCC;
use stm32l4xx_hal::gpio::Alternate;
use stm32l4xx_hal::gpio::Floating;
use stm32l4xx_hal::gpio::Input;
use stm32l4xx_hal::gpio::AF10;
use stm32l4xx_hal::gpio::PA11;
use stm32l4xx_hal::gpio::PA12;
use stm32l4xx_hal::rcc::PllConfig;
use stm32l4xx_hal::rcc::PllDivider;
use stm32l4xx_hal::{pac, prelude::*, stm32};

use usb_device::{bus::UsbBusAllocator, prelude::*};

use usbd_serial::SerialPort;
use usbd_serial::USB_CLASS_CDC;

pub struct Peripheral {
    pub usb: stm32::USB,
    pub pin_dm: PA11<Alternate<AF10, Input<Floating>>>,
    pub pin_dp: PA12<Alternate<AF10, Input<Floating>>>,
}

unsafe impl Sync for Peripheral {}

unsafe impl UsbPeripheral for Peripheral {
    const REGISTERS: *const () = stm32::USB::ptr() as *const ();

    const DP_PULL_UP_FEATURE: bool = true; // internal pull-up supported by stm32l*
                                           // stm32l433.pdf: p.69
    const EP_MEMORY: *const () = 0x4000_6C00 as _;
    const EP_MEMORY_SIZE: usize = 1024;
    const EP_MEMORY_ACCESS_2X16: bool = true;

    fn enable() {
        let rcc = unsafe { &*RCC::ptr() };

        cortex_m::interrupt::free(|_| {
            // Enable USB peripheral
            rcc.apb1enr1.modify(|_, w| w.usbfsen().set_bit());

            // Reset USB peripheral
            rcc.apb1rstr1
                .modify(|r, w| unsafe { w.bits(r.bits() | (1u32 << 26)) });
            rcc.apb1rstr1
                .modify(|r, w| unsafe { w.bits(r.bits() & !(1u32 << 26)) });
        });
    }

    fn startup_delay() {
        // There is a chip specific startup delay. For STM32F103xx it's 1µs and this should wait for
        // at least that long.
        cortex_m::asm::delay(72);
    }
}

fn enable_crs() {
    let rcc = unsafe { &(*stm32::RCC::ptr()) };
    rcc.apb1enr1.modify(|_, w| w.crsen().set_bit());
    let crs = unsafe { &(*stm32::CRS::ptr()) };
    // Initialize clock recovery
    // Set autotrim enabled.
    crs.cr.modify(|_, w| w.autotrimen().set_bit());
    // Enable CR
    crs.cr.modify(|_, w| w.cen().set_bit());
}

/// Enables VddUSB power supply
fn enable_usb_pwr() {
    // Enable PWR peripheral
    let rcc = unsafe { &(*stm32::RCC::ptr()) };
    rcc.apb1enr1.modify(|_, w| w.pwren().set_bit());

    // Enable VddUSB
    let pwr = unsafe { &*stm32::PWR::ptr() };
    pwr.cr2.modify(|_, w| w.usv().set_bit());
}

#[app(device = stm32l4xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_bus: &'static UsbBusAllocator<stm32_usbd::UsbBus<Peripheral>>,
        usb_device: UsbDevice<'static, stm32_usbd::UsbBus<Peripheral>>,
        serial: SerialPort<'static, stm32_usbd::UsbBus<Peripheral>>,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        static mut USB_BUS: Option<UsbBusAllocator<UsbBus<Peripheral>>> = None;

        // RTT handler
        rtt_init_print!();

        // Alias peripherals
        let dp: pac::Peripherals = ctx.device;

        rprintln!("Initializing peripherals");
        let mut rcc = dp.RCC.constrain();
        let mut flash = dp.FLASH.constrain();
        let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

        {
            // set USB 48Mhz clock src
            // can be configured only before PLL enable
            let _rcc = unsafe { &*stm32::RCC::ptr() };

            _rcc.pllcfgr.modify(|_, w| unsafe {
                w.pllq()
                    .bits(0b00) // /2
                    .pllqen()
                    .set_bit() // enable PLLQ
            });

            // PLLQ -> CLK48MHz
            unsafe { _rcc.ccipr.modify(|_, w| w.clk48sel().bits(0b10)) };
            // HSI48 -> CLK48MHz
            //unsafe { _rcc.ccipr.modify(|_, w| w.clk48sel().bits(0b00)) };
        }

        rcc.cfgr
            .hsi48(true)
            .hse(
                12.mhz(),
                stm32l4xx_hal::rcc::CrystalBypass::Disable,
                stm32l4xx_hal::rcc::ClockSecuritySystem::Enable,
            )
            .sysclk_with_pll(24.mhz(), PllConfig::new(1, 8, PllDivider::Div4))
            .pll_source(stm32l4xx_hal::rcc::PllSource::HSE)
            .pclk1(24.mhz())
            .pclk2(24.mhz())
            .freeze(&mut flash.acr, &mut pwr);

        rprintln!("Enable USB CRS");
        enable_crs();

        rprintln!("Enable USB power");
        enable_usb_pwr();

        // USB pins
        let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
        let usb = Peripheral {
            usb: dp.USB,
            pin_dm: gpioa.pa11.into_af10(&mut gpioa.moder, &mut gpioa.afrh),
            pin_dp: gpioa.pa12.into_af10(&mut gpioa.moder, &mut gpioa.afrh),
        };
        *USB_BUS = Some(UsbBus::new(usb));

        let serial = SerialPort::new(USB_BUS.as_ref().unwrap());

        let usb_device =
            UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST")
                .device_class(USB_CLASS_CDC)
                .build();

        rprintln!("Defining late resources...");

        init::LateResources {
            usb_bus: USB_BUS.as_ref().unwrap(),
            usb_device,
            //usb_hid,
            serial,
        }
    }

    #[idle(resources = [usb_device, serial])]
    fn idle(_: idle::Context) -> ! {
        let _rcc = unsafe { &*stm32::RCC::ptr() };
        let pwr = unsafe { &*stm32::PWR::ptr() };
        // HSI48 -> CLK48MHz
        let mut _a = _rcc.apb1enr1.read().pwren().bits();
        let mut _b = pwr.cr2.read().usv().bits();

        loop {
            _a = !_a;
            _b = !_b;
            cortex_m::asm::nop();
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = USB, resources = [usb_device, serial])]
    fn usb_handler(ctx: usb_handler::Context) {
        rprintln!("USB interrupt received.");

        let dev = ctx.resources.usb_device;
        let serial = ctx.resources.serial;

        // USB dev poll only in the interrupt handler
        dev.poll(&mut [serial]);
    }
};
