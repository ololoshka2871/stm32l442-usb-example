#![no_std]
#![no_main]

use cortex_m_log::modes::InterruptOk;
use cortex_m_log::printer::semihosting::hio::HStdout;
use cortex_m_log::printer::{Printer, Semihosting, semihosting};
use panic_halt as _;
use rtic::app;

use stm32_usbd::{UsbBus, UsbPeripheral};
use stm32l4xx_hal::device::RCC;
use stm32l4xx_hal::gpio::{Alternate, Floating, Input, AF10, PA11, PA12};
use stm32l4xx_hal::rcc::{PllConfig, PllDivider};
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

    // internal pull-up supported by stm32l*
    const DP_PULL_UP_FEATURE: bool = true;

    // USB memory region stm32l433.pdf: p.69
    const EP_MEMORY: *const () = 0x4000_6C00 as _;

    // 0x4000_6C00 - 0x4000_6FFF
    const EP_MEMORY_SIZE: usize = 1024;
    const EP_MEMORY_ACCESS_2X16: bool = true;

    fn enable() {
        let crs = unsafe { &(*stm32::CRS::ptr()) };
        let rcc = unsafe { &*RCC::ptr() };
        let pwr = unsafe { &*stm32::PWR::ptr() };

        cortex_m::interrupt::free(|_| {
            // enable crs
            rcc.apb1enr1.modify(|_, w| w.crsen().set_bit());

            // Initialize clock recovery
            // Set autotrim enabled.
            crs.cr.modify(|_, w| w.autotrimen().set_bit());
            // Enable CR
            crs.cr.modify(|_, w| w.cen().set_bit());

            //-------------------------------------------------
            // Disable USB power isolation

            // Enable PWR peripheral
            rcc.apb1enr1.modify(|_, w| w.pwren().set_bit());

            // enable montoring 1.2v
            pwr.cr2.modify(|_, w| w.pvme1().set_bit());

            // wait bit clear
            while !pwr.sr2.read().pvmo1().bit_is_clear() {
                cortex_m::asm::delay(1);
            }

            // disable monitoring
            pwr.cr2.modify(|_, w| w.pvme1().clear_bit());

            // Enable VddUSB
            pwr.cr2.modify(|_, w| w.usv().set_bit());

            //-------------------------------------------------

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

#[app(device = stm32l4xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_bus: &'static UsbBusAllocator<stm32_usbd::UsbBus<Peripheral>>,
        usb_device: UsbDevice<'static, stm32_usbd::UsbBus<Peripheral>>,
        serial: SerialPort<'static, stm32_usbd::UsbBus<Peripheral>>,
        semihosting: Semihosting<InterruptOk, HStdout>,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        static mut USB_BUS: Option<UsbBusAllocator<UsbBus<Peripheral>>> = None;

        let mut semihosting = semihosting::InterruptOk::<_>::stdout().unwrap();

        // Alias peripherals
        let dp: pac::Peripherals = ctx.device;

        let mut rcc = dp.RCC.constrain();
        let mut flash = dp.FLASH.constrain();
        let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);


        semihosting.println(format_args!("Enable PLLQ for USB 48Mhz..."));
        {
            // set USB 48Mhz clock src to PLLQ
            // can be configured only before PLL enable
            let _rcc = unsafe { &*stm32::RCC::ptr() };

            _rcc.pllcfgr.modify(|_, w| unsafe {
                w.pllq()
                    .bits(0b00) // PLLQ = PLL/2
                    .pllqen()
                    .set_bit() // enable PLLQ
            });

            // PLLQ -> CLK48MHz
            unsafe { _rcc.ccipr.modify(|_, w| w.clk48sel().bits(0b10)) };
        }

        semihosting.println(format_args!("Configuring PLL..."));
        rcc.cfgr
            .hse(
                12.mhz(), // onboard crystall
                stm32l4xx_hal::rcc::CrystalBypass::Disable,
                stm32l4xx_hal::rcc::ClockSecuritySystem::Enable,
            )
            .sysclk_with_pll(
                24.mhz(),                               // CPU clock
                PllConfig::new(1, 8, PllDivider::Div4), // PLL config
            )
            .pll_source(stm32l4xx_hal::rcc::PllSource::HSE)
            .pclk1(24.mhz())
            .pclk2(24.mhz())
            .freeze(&mut flash.acr, &mut pwr);

        // USB pins
        semihosting.println(format_args!("Creating usb allocator..."));
        let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
        let usb = Peripheral {
            usb: dp.USB,
            pin_dm: gpioa.pa11.into_af10(&mut gpioa.moder, &mut gpioa.afrh),
            pin_dp: gpioa.pa12.into_af10(&mut gpioa.moder, &mut gpioa.afrh),
        };
        *USB_BUS = Some(UsbBus::new(usb));

        semihosting.println(format_args!("Configuring USB CDCACM device..."));
        let serial = SerialPort::new(USB_BUS.as_ref().unwrap());

        semihosting.println(format_args!("Configuring USB device..."));
        let usb_device =
            UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("0123456789")
                .device_class(USB_CLASS_CDC)
                .build();

        init::LateResources {
            usb_bus: USB_BUS.as_ref().unwrap(),
            usb_device,
            serial,
            semihosting,
        }
    }

    #[idle(resources = [usb_device, serial])]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = USB, resources = [usb_device, serial])]
    fn usb_handler(ctx: usb_handler::Context) {
        let dev = ctx.resources.usb_device;
        let serial = ctx.resources.serial;

        // USB dev poll only in the interrupt handler
        if dev.poll(&mut [serial]) {
            let mut buf = [0u8; 64];

            match serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    // Echo back in upper case
                    for c in buf[0..count].iter_mut() {
                        if 0x61 <= *c && *c <= 0x7a {
                            *c &= !0x20;
                        }
                    }

                    let mut write_offset = 0;
                    while write_offset < count {
                        match serial.write(&buf[write_offset..count]) {
                            Ok(len) if len > 0 => {
                                write_offset += len;
                            }
                            _ => {}
                        }
                    }
                }
                _ => {}
            }
        }
    }
};
