//! # General Purpose Input / Output
//!
//! To use the GPIO pins, you first need to configure the GPIO port (GPIOA, GPIOB, ...) that you
//! are interested in. This is done using the [`GpioExt::split`] function.
//!
//! ```
//! let dp = pac::Peripherals::take().unwrap();
//! let rcc = dp.RCC.constrain();
//!
//! let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
//! ```
//!
//! The resulting [Parts](gpioa::Parts) struct contains one field for each pin, as well as some
//! shared registers. Every pin type is a specialized version of generic [pin](Pin) struct.
//!
//! To use a pin, first use the relevant `into_...` method of the [pin](Pin).
//!
//! ```rust
//! let pa0 = gpioa.pa0.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
//! ```
//!
//! And finally, you can use the functions from the [InputPin] or [OutputPin] traits in
//! `embedded_hal`
//!
//! For a complete example, see [examples/toggle.rs]
//!
//! ## Pin Configuration
//!
//! ### Mode
//!
//! Each GPIO pin can be set to various modes by corresponding `into_...` method:
//!
//! - **Input**: The output buffer is disabled and the schmitt trigger input is activated
//! - **Output**: Both the output buffer and the schmitt trigger input is enabled
//!     - **PushPull**: Output which either drives the pin high or low
//!     - **OpenDrain**: Output which leaves the gate floating, or pulls it to ground in drain
//!     mode. Can be used as an input in the `open` configuration
//! - **Alternate**: Pin mode required when the pin is driven by other peripherals. The schmitt
//! trigger input is activated. The Output buffer is automatically enabled and disabled by
//! peripherals. Output behavior is same as the output mode
//!     - **PushPull**: Output which either drives the pin high or low
//!     - **OpenDrain**: Output which leaves the gate floating, or pulls it to ground in drain
//!     mode
//! - **Analog**: Pin mode required for ADC, DAC, OPAMP, and COMP peripherals. It is also suitable
//! for minimize energy consumption as the output buffer and the schmitt trigger input is disabled
//!
//! ### Output Speed
//!
//! Output speed (slew rate) for each pin is selectable from low, medium, and high by calling
//! [`set_speed`](Pin::set_speed) method. Refer to the device datasheet for specifications for each
//!  speed.
//!
//! ### Internal Resistor
//!
//! Weak internal pull-up and pull-down resistors for each pin is configurable by calling
//! [`set_internal_resistor`](Pin::set_internal_resistor) method. `into_..._input` methods are also
//! available for convenience.
//!
//! [InputPin]: embedded_hal::digital::v2::InputPin
//! [OutputPin]: embedded_hal::digital::v2::OutputPin
//! [examples/toggle.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.9.0/examples/toggle.rs

use core::{convert::Infallible, marker::PhantomData};

enum Toggle {
    On,
    Off,
}

use crate::embedded_hal::digital::v2::OutputPin;

use crate::hal::digital::v2::{toggleable, InputPin, StatefulOutputPin};

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The Parts to split the GPIO peripheral into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    fn split(self) -> Self::Parts;
}

/// GPIO Register interface traits private to this module
mod private {
    pub trait GpioRegExt {
        fn is_low(&self, i: u8) -> bool;
        fn is_set_low(&self, i: u8) -> bool;
        fn set_high(&self, i: u8);
        fn set_low(&self, i: u8);
    }

    pub trait MODER {
        fn input(&mut self, i: u8);
        fn output(&mut self, i: u8);
        fn alternate(&mut self, i: u8);
        fn analog(&mut self, i: u8);
    }

    pub trait OTYPER {
        fn push_pull(&mut self, i: u8);
        fn open_drain(&mut self, i: u8);
    }

    pub trait OSPEEDR {
        fn low(&mut self, i: u8);
        fn medium(&mut self, i: u8);
        fn high(&mut self, i: u8);
    }

    pub trait PUPDR {
        fn floating(&mut self, i: u8);
        fn pull_up(&mut self, i: u8);
        fn pull_down(&mut self, i: u8);
    }

    pub trait AfR {
        fn afx(&mut self, i: u8, x: u8);
    }

    pub trait Gpio {
        type Reg: GpioRegExt + ?Sized;

        fn ptr(&self) -> *const Self::Reg;
        fn port_index(&self) -> u8;
    }
}

use private::{AfR, GpioRegExt, MODER, OSPEEDR, OTYPER, PUPDR};

/// Marker traits used in this module
pub mod marker {
    /// Marker trait for GPIO ports
    pub trait Gpio: super::private::Gpio {}

    /// Marker trait for compile time defined GPIO ports
    pub trait GpioStatic: Gpio {
        /// Associated MODER register
        type MODER: super::MODER;
        /// Associated OTYPER register
        type OTYPER: super::OTYPER;
        /// Associated OSPEEDR register
        type OSPEEDR: super::OSPEEDR;
        /// Associated PUPDR register
        type PUPDR: super::PUPDR;
    }

    /// Marker trait for pin number
    pub trait Index {
        #[doc(hidden)]
        fn index(&self) -> u8;
    }

    /// Marker trait for readable pin modes
    pub trait Readable {}

    /// Marker trait for slew rate configurable pin modes
    pub trait OutputSpeed {}

    /// Marker trait for active pin modes
    pub trait Active {}

    /// Marker trait for pins with alternate function `A` mapping
    pub trait IntoAf<const A: u8> {
        /// Associated AFR register
        type AFR: super::AfR;
    }
}

/// Runtime defined GPIO port (type state)
#[derive(Debug)]
pub struct Gpiox {
    ptr: *const dyn GpioRegExt,
    index: u8,
}

#[cfg(feature = "defmt")]
impl defmt::Format for Gpiox {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "Gpiox {{ ptr: GpioRegExt , index: {:?} }}", self.index);
    }
}

// # SAFETY
// As Gpiox uses `dyn GpioRegExt` pointer internally, `Send` is not auto-implemented.
// But since GpioExt does only do atomic operations without side-effects we can assume
// that it safe to `Send` this type.
unsafe impl Send for Gpiox {}

// # SAFETY
// As Gpiox uses `dyn GpioRegExt` pointer internally, `Sync` is not auto-implemented.
// But since GpioExt does only do atomic operations without side-effects we can assume
// that it safe to `Send` this type.
unsafe impl Sync for Gpiox {}

impl private::Gpio for Gpiox {
    type Reg = dyn GpioRegExt;

    fn ptr(&self) -> *const Self::Reg {
        self.ptr
    }

    fn port_index(&self) -> u8 {
        self.index
    }
}

impl marker::Gpio for Gpiox {}

/// Runtime defined pin number (type state)
// TODO(Sh3Rm4n): If the pin number wouldn't be runtime defined, the implementation for all
// statically defined pins would be much easier (and withless overhead). What could be the
// solution?
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ux(u8);

impl marker::Index for Ux {
    fn index(&self) -> u8 {
        self.0
    }
}

/// Compile time defined pin number (type state)
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct U<const X: u8>;

impl<const X: u8> marker::Index for U<X> {
    #[inline(always)]
    fn index(&self) -> u8 {
        X
    }
}

/// Input mode (type state)
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Input;
/// Output mode (type state)
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Output<Otype>(PhantomData<Otype>);
/// Alternate function (type state)
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Alternate<Otype, const AF: u8>(PhantomData<Otype>);
/// Analog mode (type state)
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Analog;

/// Push-pull output (type state)
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PushPull;
/// Open-drain output (type state)
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OpenDrain;

impl marker::Readable for Input {}
impl marker::Readable for Output<OpenDrain> {}
impl<Otype> marker::OutputSpeed for Output<Otype> {}
impl<Otype, const AF: u8> marker::OutputSpeed for Alternate<Otype, AF> {}
impl marker::Active for Input {}
impl<Otype> marker::Active for Output<Otype> {}
impl<Otype, const AF: u8> marker::Active for Alternate<Otype, AF> {}

/// Slew rate configuration
#[derive(Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Speed {
    /// Low speed
    Low,
    /// High speed
    High,
}

/// Internal pull-up and pull-down resistor configuration
#[derive(Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Resistor {
    /// Floating
    Floating,
    /// Pulled up
    PullUp,
    /// Pulled down
    PullDown,
}

/// GPIO interrupt trigger edge selection
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Edge {
    /// Rising edge of voltage
    Rising,
    /// Falling edge of voltage
    Falling,
    /// Rising and falling edge of voltage
    RisingFalling,
}

/// Generic pin
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Pin<Gpio, Index, Mode> {
    pub(crate) gpio: Gpio,
    pub(crate) index: Index,
    _mode: PhantomData<Mode>,
}

// Make all GPIO peripheral trait extensions sealable.
impl<Gpio, Index, Mode> crate::private::Sealed for Pin<Gpio, Index, Mode> {}

/// Fully erased pin
///
/// This moves the pin type information to be known
/// at runtime, and erases the specific compile time type of the GPIO.
/// The only compile time information of the GPIO pin is it's Mode.
///
/// See [examples/gpio_erased.rs] as an example.
///
/// [examples/gpio_erased.rs]: https://github.com/stm32-rs/stm32f3xx-hal/blob/v0.9.0/examples/gpio_erased.rs
pub type PXx<Mode> = Pin<Gpiox, Ux, Mode>;

impl<Gpio, Mode, const X: u8> Pin<Gpio, U<X>, Mode> {
    /// Erases the pin number from the type
    ///
    /// This is useful when you want to collect the pins into an array where you
    /// need all the elements to have the same type
    pub fn downgrade(self) -> Pin<Gpio, Ux, Mode> {
        Pin {
            gpio: self.gpio,
            index: Ux(X),
            _mode: self._mode,
        }
    }
}

impl<Gpio, Mode> Pin<Gpio, Ux, Mode>
where
    Gpio: marker::GpioStatic,
    Gpio::Reg: 'static + Sized,
{
    /// Erases the port letter from the type
    ///
    /// This is useful when you want to collect the pins into an array where you
    /// need all the elements to have the same type
    pub fn downgrade(self) -> PXx<Mode> {
        PXx {
            gpio: Gpiox {
                ptr: self.gpio.ptr(),
                index: self.gpio.port_index(),
            },
            index: self.index,
            _mode: self._mode,
        }
    }
}

impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode> {
    fn into_mode<NewMode>(self) -> Pin<Gpio, Index, NewMode> {
        Pin {
            gpio: self.gpio,
            index: self.index,
            _mode: PhantomData,
        }
    }
}

impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode>
where
    Gpio: marker::GpioStatic,
    Index: marker::Index,
{
    /// Configures the pin to operate as an input pin
    pub fn into_input(self, moder: &mut Gpio::MODER) -> Pin<Gpio, Index, Input> {
        moder.input(self.index.index());
        self.into_mode()
    }

    /// Convenience method to configure the pin to operate as an input pin
    /// and set the internal resistor floating
    pub fn into_floating_input(
        self,
        moder: &mut Gpio::MODER,
        pupdr: &mut Gpio::PUPDR,
    ) -> Pin<Gpio, Index, Input> {
        moder.input(self.index.index());
        pupdr.floating(self.index.index());
        self.into_mode()
    }

    /// Convenience method to configure the pin to operate as an input pin
    /// and set the internal resistor pull-up
    pub fn into_pull_up_input(
        self,
        moder: &mut Gpio::MODER,
        pupdr: &mut Gpio::PUPDR,
    ) -> Pin<Gpio, Index, Input> {
        moder.input(self.index.index());
        pupdr.pull_up(self.index.index());
        self.into_mode()
    }

    /// Convenience method to configure the pin to operate as an input pin
    /// and set the internal resistor pull-down
    pub fn into_pull_down_input(
        self,
        moder: &mut Gpio::MODER,
        pupdr: &mut Gpio::PUPDR,
    ) -> Pin<Gpio, Index, Input> {
        moder.input(self.index.index());
        pupdr.pull_down(self.index.index());
        self.into_mode()
    }

    /// Configures the pin to operate as a push-pull output pin
    pub fn into_push_pull_output(
        self,
        moder: &mut Gpio::MODER,
        otyper: &mut Gpio::OTYPER,
    ) -> Pin<Gpio, Index, Output<PushPull>> {
        moder.output(self.index.index());
        otyper.push_pull(self.index.index());
        self.into_mode()
    }

    /// Configures the pin to operate as an open-drain output pin
    pub fn into_open_drain_output(
        self,
        moder: &mut Gpio::MODER,
        otyper: &mut Gpio::OTYPER,
    ) -> Pin<Gpio, Index, Output<OpenDrain>> {
        moder.output(self.index.index());
        otyper.open_drain(self.index.index());
        self.into_mode()
    }

    /// Configures the pin to operate as an analog pin, with disabled schmitt trigger.
    pub fn into_analog(
        self,
        moder: &mut Gpio::MODER,
        pupdr: &mut Gpio::PUPDR,
    ) -> Pin<Gpio, Index, Analog> {
        moder.analog(self.index.index());
        pupdr.floating(self.index.index());
        self.into_mode()
    }
}

impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode>
where
    Gpio: marker::GpioStatic,
    Index: marker::Index,
    Mode: marker::OutputSpeed,
{
    /// Set pin output slew rate
    pub fn set_speed(&mut self, ospeedr: &mut Gpio::OSPEEDR, speed: Speed) {
        match speed {
            Speed::Low => ospeedr.low(self.index.index()),
            Speed::High => ospeedr.high(self.index.index()),
        }
    }
}

impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode>
where
    Gpio: marker::GpioStatic,
    Index: marker::Index,
    Mode: marker::Active,
{
    /// Set the internal pull-up and pull-down resistor
    /// TODO: Get rid of this pupdr reference. These boards use a diffent thing
    pub fn set_internal_resistor(&mut self, pupdr: &mut Gpio::PUPDR, resistor: Resistor) {
        match resistor {
            Resistor::Floating => pupdr.floating(self.index.index()),
            Resistor::PullUp => pupdr.pull_up(self.index.index()),
            Resistor::PullDown => pupdr.pull_down(self.index.index()),
        }
    }

    /// Enables / disables the internal pull up (Provided for compatibility with other stm32 HALs)
    pub fn internal_pull_up(&mut self, pupdr: &mut Gpio::PUPDR, on: bool) {
        if on {
            pupdr.pull_up(self.index.index());
        } else {
            pupdr.floating(self.index.index());
        }
    }
}

impl<Gpio, Index, Otype> OutputPin for Pin<Gpio, Index, Output<Otype>>
where
    Gpio: marker::Gpio,
    Index: marker::Index,
{
    type Error = Infallible;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        // NOTE(unsafe) atomic write to a stateless register
        unsafe { (*self.gpio.ptr()).set_high(self.index.index()) };
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        // NOTE(unsafe) atomic write to a stateless register
        unsafe { (*self.gpio.ptr()).set_low(self.index.index()) };
        Ok(())
    }
}

impl<Gpio, Index, Mode> InputPin for Pin<Gpio, Index, Mode>
where
    Gpio: marker::Gpio,
    Index: marker::Index,
    Mode: marker::Readable,
{
    type Error = Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(!self.is_low()?)
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        // NOTE(unsafe) atomic read with no side effects
        Ok(unsafe { (*self.gpio.ptr()).is_low(self.index.index()) })
    }
}

impl<Gpio, Index, Otype> StatefulOutputPin for Pin<Gpio, Index, Output<Otype>>
where
    Gpio: marker::Gpio,
    Index: marker::Index,
{
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(!self.is_set_low()?)
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        // NOTE(unsafe) atomic read with no side effects
        Ok(unsafe { (*self.gpio.ptr()).is_set_low(self.index.index()) })
    }
}

impl<Gpio, Index, Otype> toggleable::Default for Pin<Gpio, Index, Output<Otype>>
where
    Gpio: marker::Gpio,
    Index: marker::Index,
{
}

//impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode>
//where
//    Gpio: marker::Gpio,
//    Index: marker::Index,
//    Mode: marker::Active,
//{
//    /// NVIC interrupt number of interrupt from this pin
//    ///
//    /// Used to unmask / enable the interrupt with [`cortex_m::peripheral::NVIC::unmask()`].
//    /// This is also useful for all other [`cortex_m::peripheral::NVIC`] functions.
//    // TODO(Sh3rm4n): It would be cool to have this either const or have a const function.
//    // But this is currenlty not possible, because index() is runtime defined.
//    pub fn interrupt(&self) -> Interrupt {
//        match self.index.index() {
//            0 => Interrupt::EXTI0,
//            1 => Interrupt::EXTI1,
//            #[cfg(feature = "svd-f373")]
//            2 => Interrupt::EXTI2_TS,
//            #[cfg(not(feature = "svd-f373"))]
//            2 => Interrupt::EXTI2_TSC,
//            3 => Interrupt::EXTI3,
//            4 => Interrupt::EXTI4,
//            #[cfg(feature = "svd-f373")]
//            5..=9 => Interrupt::EXTI5_9,
//            #[cfg(not(feature = "svd-f373"))]
//            5..=9 => Interrupt::EXTI9_5,
//            10..=15 => Interrupt::EXTI15_10,
//            _ => crate::unreachable!(),
//        }
//    }

//    /// Generate interrupt on rising edge, falling edge, or both
//    pub fn trigger_on_edge(&mut self, exti: &mut EXTI, edge: Edge) {
//        const BITWIDTH: u8 = 1;
//        let index = self.index.index();
//        let (rise, fall) = match edge {
//            Edge::Rising => (true as u32, false as u32),
//            Edge::Falling => (false as u32, true as u32),
//            Edge::RisingFalling => (true as u32, true as u32),
//        };
//        // SAFETY: Unguarded write to the register, but behind a &mut
//        unsafe {
//            crate::modify_at!(reg_for_cpu!(exti, rtsr), BITWIDTH, index, rise);
//            crate::modify_at!(reg_for_cpu!(exti, ftsr), BITWIDTH, index, fall);
//        }
//    }

//    /// Configure external interrupts from this pin
//    ///
//    /// # Note
//    ///
//    /// Remeber to also configure the interrupt pin on
//    /// the SysCfg site, with [`crate::syscfg::SysCfg::select_exti_interrupt_source()`]
//    pub fn configure_interrupt(&mut self, exti: &mut EXTI, enable: impl Into<Toggle>) {
//        const BITWIDTH: u8 = 1;

//        let enable: Toggle = enable.into();
//        let enable: bool = enable.into();

//        let index = self.index.index();
//        let value = u32::from(enable);
//        // SAFETY: Unguarded write to the register, but behind a &mut
//        unsafe { crate::modify_at!(reg_for_cpu!(exti, imr), BITWIDTH, index, value) };
//    }

//    /// Enable external interrupts from this pin
//    ///
//    /// # Note
//    ///
//    /// Remeber to also configure the interrupt pin on
//    /// the SysCfg site, with [`crate::syscfg::SysCfg::select_exti_interrupt_source()`]
//    pub fn enable_interrupt(&mut self, exti: &mut EXTI) {
//        self.configure_interrupt(exti, Toggle::On)
//    }

//    /// Disable external interrupts from this pin
//    pub fn disable_interrupt(&mut self, exti: &mut EXTI) {
//        self.configure_interrupt(exti, Toggle::Off)
//    }

//    /// Clear the interrupt pending bit for this pin
//    pub fn clear_interrupt(&mut self) {
//        // SAFETY: Atomic write to register without side-effects.
//        unsafe { reg_for_cpu!((*EXTI::ptr()), pr).write(|w| w.bits(1 << self.index.index())) };
//    }

//    /// Reads the interrupt pending bit for this pin
//    pub fn is_interrupt_pending(&self) -> bool {
//        // SAFETY: Atomic write to register without side-effects.
//        unsafe { reg_for_cpu!((*EXTI::ptr()), pr).read().bits() & (1 << self.index.index()) != 0 }
//    }
//}

impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode>
where
    Gpio: marker::GpioStatic,
    Index: marker::Index,
{
    /// Configures the pin to operate as an alternate function push-pull output pin
    pub fn into_af_push_pull<const A: u8>(
        self,
        moder: &mut Gpio::MODER,
        otyper: &mut Gpio::OTYPER,
        afr: &mut <Self as marker::IntoAf<A>>::AFR,
    ) -> Pin<Gpio, Index, Alternate<PushPull, A>>
    where
        Self: marker::IntoAf<A>,
    {
        moder.alternate(self.index.index());
        otyper.push_pull(self.index.index());
        afr.afx(self.index.index(), A);
        self.into_mode()
    }

    /// Configures the pin to operate as an alternate function open-drain output pin
    pub fn into_af_open_drain<const A: u8>(
        self,
        moder: &mut Gpio::MODER,
        otyper: &mut Gpio::OTYPER,
        afr: &mut <Self as marker::IntoAf<A>>::AFR,
    ) -> Pin<Gpio, Index, Alternate<OpenDrain, A>>
    where
        Self: marker::IntoAf<A>,
    {
        moder.alternate(self.index.index());
        otyper.open_drain(self.index.index());
        afr.afx(self.index.index(), A);
        self.into_mode()
    }
}

macro_rules! af {
    ($i:literal, $AFi:ident, $into_afi_push_pull:ident, $into_afi_open_drain:ident) => {
        #[doc = concat!("Alternate function ", $i, " (type state)" )]
        pub type $AFi<Otype> = Alternate<Otype, $i>;

        impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode>
        where
            Self: marker::IntoAf<$i>,
            Gpio: marker::GpioStatic,
            Index: marker::Index,
        {
            /// Configures the pin to operate as an alternate function push-pull output pin
            #[deprecated(since = "0.9.0", note = "Will be removed with the next version. Use `into_af_push_pull()` instead")]
            pub fn $into_afi_push_pull(
                self,
                moder: &mut Gpio::MODER,
                otyper: &mut Gpio::OTYPER,
                afr: &mut <Self as marker::IntoAf<$i>>::AFR,
            ) -> Pin<Gpio, Index, $AFi<PushPull>> {
                self.into_af_push_pull::<$i>(moder, otyper, afr)
            }

            /// Configures the pin to operate as an alternate function open-drain output pin
            #[deprecated(since = "0.9.0", note = "Will be removed with the next version. Use `into_af_open_drain()` instead")]
            pub fn $into_afi_open_drain(
                self,
                moder: &mut Gpio::MODER,
                otyper: &mut Gpio::OTYPER,
                afr: &mut <Self as marker::IntoAf<$i>>::AFR,
            ) -> Pin<Gpio, Index, $AFi<OpenDrain>> {
                self.into_af_open_drain::<$i>(moder, otyper, afr)
            }
        }
    };

    ([$($i:literal),+ $(,)?]) => {
        paste::paste! {
            $(
                af!($i, [<AF $i>],  [<into_af $i _push_pull>],  [<into_af $i _open_drain>]);
            )+
        }
    };
}

af!([0, 1, 2, 3, 4, 5, 6, 7]);

// Allow unused because it is used but only in a macro (so it hasn't been expanded)
#[allow(unused)]
macro_rules! gpio_trait {
    ([$($gpioy:ident),+ $(,)?]) => {
        $(
            impl GpioRegExt for crate::pac::$gpioy::RegisterBlock {
                #[inline(always)]
                fn is_low(&self, i: u8) -> bool {
                    self.idr.read().bits() & (1 << i) == 0
                }

                #[inline(always)]
                fn is_set_low(&self, i: u8) -> bool {
                    self.odr.read().bits() & (1 << i) == 0
                }

                #[inline(always)]
                fn set_high(&self, i: u8) {
                    // NOTE(unsafe, write) atomic write to a stateless register
                    unsafe { self.bsrr.write(|w| w.bits(1 << i)) };
                }

                #[inline(always)]
                fn set_low(&self, i: u8) {
                    // NOTE(unsafe, write) atomic write to a stateless register
                    unsafe { self.bsrr.write(|w| w.bits(1 << (16 + i))) };
                }
            }
        )+
    };
}

/// Implement private::{Moder, Ospeedr, Otyper, Pupdr} traits for each opaque register structs
#[allow(unused)]
macro_rules! r_trait {
    (
        ($GPIOX:ident, $gpioy:ident::$xr:ident::$enum:ident, $bitwidth:expr);
        impl $Xr:ident for $XR:ty {
            $(
                fn $fn:ident { $VARIANT:ident }
            )+
        }
    ) => {
        impl $Xr for $XR {
            $(
                #[inline]
                fn $fn(&mut self, i: u8) {
                    let value = $gpioy::$xr::$enum::$VARIANT as u32;
                    unsafe { crate::modify_at!((*$GPIOX::ptr()).$xr, $bitwidth, i, value) };
                }
            )+
        }
    };
}

macro_rules! gpio {
    ({
        GPIO: $GPIOX:ident,
        gpio: $gpiox:ident,
        Gpio: $Gpiox:ident,
        port_index: $port_index:literal,
        gpio_mapped: $gpioy:ident,
        partially_erased_pin: $PXx:ident,
        pins: [$(
            $i:literal => (
                $PXi:ident, $pxi:ident, $MODE:ty, $AFR:ident, [$($af:literal),*],
            ),
        )+],
    }) => {
        #[doc = concat!("GPIO port ", stringify!($GPIOX), " (type state)")]
        #[derive(Debug)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        pub struct $Gpiox;

        impl private::Gpio for $Gpiox {
            type Reg = crate::pac::$gpioy::RegisterBlock;

            #[inline(always)]
            fn ptr(&self) -> *const Self::Reg {
                crate::pac::$GPIOX::ptr()
            }

            #[inline(always)]
            fn port_index(&self) -> u8 {
                $port_index
            }
        }

        impl marker::Gpio for $Gpiox {}

        impl marker::GpioStatic for $Gpiox {
            type MODER = $gpiox::MODER;
            type OTYPER = $gpiox::OTYPER;
            type OSPEEDR = $gpiox::OSPEEDR;
            type PUPDR = $gpiox::PUPDR;
        }

        $(
            #[doc = concat!("Pin ", stringify!($PXi))]
            pub type $PXi<Mode> = Pin<$Gpiox, U<$i>, Mode>;

            $(
                impl<Mode> marker::IntoAf<$af> for $PXi<Mode> {
                    type AFR = $gpiox::$AFR;
                }
            )*
        )+

        #[doc = concat!("Partially erased pin for ", stringify!($GPIOX))]
        pub type $PXx<Mode> = Pin<$Gpiox, Ux, Mode>;

        #[doc = concat!("All Pins and associated registers for GPIO port ", stringify!($GPIOX))]
        pub mod $gpiox {
            use core::marker::PhantomData;

            use crate::{
                pac::{$gpioy, $GPIOX},
                rcc::{AHB, Enable, Reset},
            };

            use super::{Afr, $Gpiox, GpioExt, Moder, Ospeedr, Otyper, Pupdr, U};

            #[allow(unused_imports)]
            use super::{
                Input, Output, Analog, PushPull, OpenDrain,
                AF0, AF1, AF2, AF3, AF4, AF5, AF6, AF7, AF8, AF9, AF10, AF11, AF12, AF13, AF14, AF15,
            };

            pub use super::{
                $PXx,
                $(
                    $PXi,
                )+
            };

            /// GPIO parts
            pub struct Parts {
                /// Opaque AFRH register
                pub afrh: AFRH,
                /// Opaque AFRL register
                pub afrl: AFRL,
                /// Opaque MODER register
                pub moder: MODER,
                /// Opaque OSPEEDR register
                pub ospeedr: OSPEEDR,
                /// Opaque OTYPER register
                pub otyper: OTYPER,
                /// Opaque PUPDR register
                pub pupdr: PUPDR,
                $(
                    #[doc = concat!("Pin ", stringify!($PXi))]
                    pub $pxi: $PXi<$MODE>,
                )+
            }

            impl GpioExt for $GPIOX {
                type Parts = Parts;

                fn split(self, ahb: &mut AHB) -> Parts {
                    <$GPIOX>::enable(ahb);
                    <$GPIOX>::reset(ahb);

                    Parts {
                        afrh: AFRH(()),
                        afrl: AFRL(()),
                        moder: MODER(()),
                        ospeedr: OSPEEDR(()),
                        otyper: OTYPER(()),
                        pupdr: PUPDR(()),
                        $(
                            $pxi: $PXi {
                                gpio: $Gpiox,
                                index: U::<$i>,
                                _mode: PhantomData,
                            },
                        )+
                    }
                }
            }

            /// Opaque AFRH register
            pub struct AFRH(());

            impl Afr for AFRH {
                #[inline]
                fn afx(&mut self, i: u8, x: u8) {
                    const BITWIDTH: u8 = 4;
                    unsafe { crate::modify_at!((*$GPIOX::ptr()).afrh, BITWIDTH, i - 8, x as u32) };
                }
            }

            /// Opaque AFRL register
            pub struct AFRL(());

            impl Afr for AFRL {
                #[inline]
                fn afx(&mut self, i: u8, x: u8) {
                    const BITWIDTH: u8 = 4;
                    unsafe { crate::modify_at!((*$GPIOX::ptr()).afrl, BITWIDTH, i, x as u32) };
                }
            }

            /// Opaque MODER register
            pub struct MODER(());

            r_trait! {
                ($GPIOX, $gpioy::moder::MODER15_A, 2);
                impl Moder for MODER {
                    fn input { INPUT }
                    fn output { OUTPUT }
                    fn alternate { ALTERNATE }
                    fn analog { ANALOG }
                }
            }

            /// Opaque OSPEEDR register
            pub struct OSPEEDR(());

            r_trait! {
                ($GPIOX, $gpioy::ospeedr::OSPEEDR15_A, 2);
                impl Ospeedr for OSPEEDR {
                    fn low { LOWSPEED }
                    fn medium { MEDIUMSPEED }
                    fn high { HIGHSPEED }
                }
            }

            /// Opaque OTYPER register
            pub struct OTYPER(());

            r_trait! {
                ($GPIOX, $gpioy::otyper::OT15_A, 1);
                impl Otyper for OTYPER {
                    fn push_pull { PUSHPULL }
                    fn open_drain { OPENDRAIN }
                }
            }

            /// Opaque PUPDR register
            pub struct PUPDR(());

            r_trait! {
                ($GPIOX, $gpioy::pupdr::PUPDR15_A, 2);
                impl Pupdr for PUPDR {
                    fn floating { FLOATING }
                    fn pull_up { PULLUP }
                    fn pull_down { PULLDOWN }
                }
            }
        }
    };

    ({
        pacs: $pacs:tt,
        ports: [$(
            {
                port: ($X:ident/$x:ident, $port_index:literal, $gpioy:ident),
                pins: [$(
                    $i:literal => {
                        reset: $MODE:ty,
                        afr: $LH:ident,
                        af: [$($af:literal),*]
                    },
                )+],
            },
        )+],
    }) => {
        paste::paste! {
            gpio_trait!($pacs);
            $(
                gpio!({
                    GPIO: [<GPIO $X>],
                    gpio: [<gpio $x>],
                    Gpio: [<Gpio $x>],
                    port_index: $port_index,
                    gpio_mapped: $gpioy,
                    partially_erased_pin: [<P $X x>],
                    pins: [$(
                        $i => (
                            [<P $X $i>], [<p $x $i>], $MODE, [<AFR $LH>], [$($af),*],
                        ),
                    )+],
                });
            )+
        }
    };
}
// auto-generated using codegen
// STM32CubeMX DB release: DB.6.0.10

gpio!({
    pacs: [prt0, prt1, prt2, prt3, prt4],
    ports: [
        {
            port: (0, 0, prt0),
            pins: [
                0 => { reset: Input, afr: L, af: [1, 3, 7, 15] },
                1 => { reset: Input, afr: L, af: [0, 1, 3, 7, 9, 15] },
                2 => { reset: Input, afr: L, af: [1, 3, 7, 8, 9, 15] },
                3 => { reset: Input, afr: L, af: [1, 3, 7, 9, 15] },
                4 => { reset: Input, afr: L, af: [3, 6, 7, 15] },
                5 => { reset: Input, afr: L, af: [1, 3, 15] },
                6 => { reset: Input, afr: L, af: [1, 3, 6, 15] },
                7 => { reset: Input, afr: L, af: [1, 3, 6, 15] },
                // 8 => { reset: Input, afr: H, af: [0, 3, 4, 5, 6, 7, 15] },
                // 9 => { reset: Input, afr: H, af: [2, 3, 4, 5, 6, 7, 9, 10, 15] },
                // 10 => { reset: Input, afr: H, af: [1, 3, 4, 5, 6, 7, 8, 10, 15] },
                // 11 => { reset: Input, afr: H, af: [5, 6, 7, 9, 11, 12, 15] },
                // 12 => { reset: Input, afr: H, af: [1, 5, 6, 7, 8, 9, 11, 15] },
                // 13 => { reset: AF0<PushPull>, afr: H, af: [0, 1, 3, 5, 7, 15] },
                // 14 => { reset: AF0<PushPull>, afr: H, af: [0, 3, 4, 6, 7, 15] },
                // 15 => { reset: AF0<PushPull>, afr: H, af: [0, 1, 3, 4, 6, 7, 9, 15] },
            ],
        },
        {
            port: (1, 1, prt1),
            pins: [
                0 => { reset: Input, afr: L, af: [3, 6, 15] },
                1 => { reset: Input, afr: L, af: [3, 6, 8, 15] },
                2 => { reset: Input, afr: L, af: [3, 15] },
                3 => { reset: AF0<PushPull>, afr: L, af: [0, 1, 3, 6, 7, 15] },
                4 => { reset: AF0<PushPull>, afr: L, af: [0, 1, 3, 6, 7, 10, 15] },
                5 => { reset: Input, afr: L, af: [1, 4, 6, 7, 8, 10, 15] },
                6 => { reset: Input, afr: L, af: [1, 3, 4, 7, 15] },
                7 => { reset: Input, afr: L, af: [1, 3, 4, 7, 15] },
                // 8 => { reset: Input, afr: H, af: [1, 3, 4, 7, 9, 12, 15] },
                // 9 => { reset: Input, afr: H, af: [1, 4, 6, 7, 8, 9, 15] },
                // 10 => { reset: Input, afr: H, af: [1, 3, 7, 15] },
                // 11 => { reset: Input, afr: H, af: [1, 3, 7, 15] },
                // 12 => { reset: Input, afr: H, af: [3, 4, 5, 6, 7, 15] },
                // 13 => { reset: Input, afr: H, af: [3, 5, 6, 7, 15] },
                // 14 => { reset: Input, afr: H, af: [1, 3, 5, 6, 7, 15] },
                // 15 => { reset: Input, afr: H, af: [0, 1, 2, 4, 5, 15] },
            ],
        },
        {
            port: (2, 2, prt3),
            pins: [
                0 => { reset: Input, afr: L, af: [1, 2] },
                1 => { reset: Input, afr: L, af: [1, 2] },
                2 => { reset: Input, afr: L, af: [1, 2] },
                3 => { reset: Input, afr: L, af: [1, 2, 6] },
                4 => { reset: Input, afr: L, af: [1, 2, 7] },
                5 => { reset: Input, afr: L, af: [1, 2, 3, 7] },
                6 => { reset: Input, afr: L, af: [1, 6, 7] },
                7 => { reset: Input, afr: L, af: [1, 6] },
                // 8 => { reset: Input, afr: H, af: [1] },
                // 9 => { reset: Input, afr: H, af: [1, 3, 5] },
                // 10 => { reset: Input, afr: H, af: [1, 6, 7] },
                // 11 => { reset: Input, afr: H, af: [1, 6, 7] },
                // 12 => { reset: Input, afr: H, af: [1, 6, 7] },
                // 13 => { reset: Input, afr: H, af: [4] },
                // 14 => { reset: Input, afr: H, af: [] },
                // 15 => { reset: Input, afr: H, af: [] },
            ],
        },
        {
            port: (3, 3, prt3),
            pins: [
                0 => { reset: Input, afr: L, af: [1, 2] },
                1 => { reset: Input, afr: L, af: [1, 2] },
                2 => { reset: Input, afr: L, af: [1, 2] },
                3 => { reset: Input, afr: L, af: [1, 2, 6] },
                4 => { reset: Input, afr: L, af: [1, 2, 7] },
                5 => { reset: Input, afr: L, af: [1, 2, 3, 7] },
                6 => { reset: Input, afr: L, af: [1, 6, 7] },
                7 => { reset: Input, afr: L, af: [1, 6] },
            ],
        },
        {
            port: (4, 4, prt4),
            pins: [
                0 => { reset: Input, afr: L, af: [4, 5, 6] },
                1 => { reset: Input, afr: L, af: [4, 5] },
            ],
        },
    ],
});

// gpio!([
//     P0_0: (p0_0, prt0, 0, Input<HighZ>),
//     P0_1: (p0_1, prt0, 1, Input<HighZ>),
//     P0_2: (p0_2, prt0, 2, Input<HighZ>),
//     P0_3: (p0_3, prt0, 3, Input<HighZ>),
//     P0_4: (p0_4, prt0, 4, Input<HighZ>),
//     P0_5: (p0_5, prt0, 5, Input<HighZ>),
//     P0_6: (p0_6, prt0, 6, Input<HighZ>),
//     P0_7: (p0_7, prt0, 7, Input<HighZ>),

//     P1_0: (p1_0, prt1, 0, Input<HighZ>),
//     P1_1: (p1_1, prt1, 1, Input<HighZ>),
//     P1_2: (p1_2, prt1, 2, Input<HighZ>),
//     P1_3: (p1_3, prt1, 3, Input<HighZ>),
//     P1_4: (p1_4, prt1, 4, Input<HighZ>),
//     P1_5: (p1_5, prt1, 5, Input<HighZ>),
//     P1_6: (p1_6, prt1, 6, Input<HighZ>),
//     P1_7: (p1_7, prt1, 7, Input<HighZ>),

//     P2_0: (p2_0, prt2, 0, Input<HighZ>),
//     P2_1: (p2_1, prt2, 1, Input<HighZ>),
//     P2_2: (p2_2, prt2, 2, Input<HighZ>),
//     P2_3: (p2_3, prt2, 3, Input<HighZ>),
//     P2_4: (p2_4, prt2, 4, Input<HighZ>),
//     P2_5: (p2_5, prt2, 5, Input<HighZ>),
//     P2_6: (p2_6, prt2, 6, Input<HighZ>),
//     P2_7: (p2_7, prt2, 7, Input<HighZ>),

//     P3_0: (p3_0, prt3, 0, Input<HighZ>),
//     P3_1: (p3_1, prt3, 1, Input<HighZ>),
//     P3_2: (p3_2, prt3, 2, Input<HighZ>),
//     P3_3: (p3_3, prt3, 3, Input<HighZ>),
//     P3_4: (p3_4, prt3, 4, Input<HighZ>),
//     P3_5: (p3_5, prt3, 5, Input<HighZ>),
//     P3_6: (p3_6, prt3, 6, Input<HighZ>),
//     P3_7: (p3_7, prt3, 7, Input<HighZ>),

//     P4_0: (p4_0, prt4, 0, Input<HighZ>),
//     P4_1: (p4_1, prt4, 1, Input<HighZ>),

//     // TODO: These are available on the 4200-ble but the pac doesn't know about them...
//     // P5_0: (p5_0, prt5, 0, Input<HighZ>),
//     // P5_1: (p5_1, prt5, 1, Input<HighZ>)
// ]);
