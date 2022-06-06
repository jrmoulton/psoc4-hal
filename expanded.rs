#![feature(prelude_import)]
#![no_std]
#[prelude_import]
use core::prelude::rust_2021::*;
#[macro_use]
extern crate core;
#[macro_use]
extern crate compiler_builtins;
pub use psoc4_pac as pac;
pub mod gpio {
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
    use crate::{
        embedded_hal::digital::v2::{toggleable, InputPin, OutputPin, StatefulOutputPin},
        pac::{Interrupt, EXTI},
        rcc::AHB,
    };
    use crate::hal::digital::v2::{toggleable, InputPin, StatefulOutputPin};
    /// Extension trait to split a GPIO peripheral in independent pins and registers
    pub trait GpioExt {
        /// The Parts to split the GPIO peripheral into
        type Parts;
        /// Splits the GPIO block into independent pins and registers
        fn split(self, ahb: &mut AHB) -> Self::Parts;
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
    pub struct Gpiox {
        ptr: *const dyn GpioRegExt,
        index: u8,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for Gpiox {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match *self {
                Gpiox {
                    ptr: ref __self_0_0,
                    index: ref __self_0_1,
                } => {
                    let debug_trait_builder = &mut ::core::fmt::Formatter::debug_struct(f, "Gpiox");
                    let _ = ::core::fmt::DebugStruct::field(
                        debug_trait_builder,
                        "ptr",
                        &&(*__self_0_0),
                    );
                    let _ = ::core::fmt::DebugStruct::field(
                        debug_trait_builder,
                        "index",
                        &&(*__self_0_1),
                    );
                    ::core::fmt::DebugStruct::finish(debug_trait_builder)
                }
            }
        }
    }
    unsafe impl Send for Gpiox {}
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
    pub struct Ux(u8);
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for Ux {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match *self {
                Ux(ref __self_0_0) => {
                    let debug_trait_builder = &mut ::core::fmt::Formatter::debug_tuple(f, "Ux");
                    let _ = ::core::fmt::DebugTuple::field(debug_trait_builder, &&(*__self_0_0));
                    ::core::fmt::DebugTuple::finish(debug_trait_builder)
                }
            }
        }
    }
    impl marker::Index for Ux {
        fn index(&self) -> u8 {
            self.0
        }
    }
    /// Compile time defined pin number (type state)
    pub struct U<const X: u8>;
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl<const X: u8> ::core::fmt::Debug for U<X> {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match *self {
                U => ::core::fmt::Formatter::write_str(f, "U"),
            }
        }
    }
    impl<const X: u8> marker::Index for U<X> {
        #[inline(always)]
        fn index(&self) -> u8 {
            X
        }
    }
    /// Input mode (type state)
    pub struct Input;
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for Input {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match *self {
                Input => ::core::fmt::Formatter::write_str(f, "Input"),
            }
        }
    }
    /// Output mode (type state)
    pub struct Output<Otype>(PhantomData<Otype>);
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl<Otype: ::core::fmt::Debug> ::core::fmt::Debug for Output<Otype> {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match *self {
                Output(ref __self_0_0) => {
                    let debug_trait_builder = &mut ::core::fmt::Formatter::debug_tuple(f, "Output");
                    let _ = ::core::fmt::DebugTuple::field(debug_trait_builder, &&(*__self_0_0));
                    ::core::fmt::DebugTuple::finish(debug_trait_builder)
                }
            }
        }
    }
    /// Alternate function (type state)
    pub struct Alternate<Otype, const AF: u8>(PhantomData<Otype>);
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl<Otype: ::core::fmt::Debug, const AF: u8> ::core::fmt::Debug for Alternate<Otype, AF> {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match *self {
                Alternate(ref __self_0_0) => {
                    let debug_trait_builder =
                        &mut ::core::fmt::Formatter::debug_tuple(f, "Alternate");
                    let _ = ::core::fmt::DebugTuple::field(debug_trait_builder, &&(*__self_0_0));
                    ::core::fmt::DebugTuple::finish(debug_trait_builder)
                }
            }
        }
    }
    /// Analog mode (type state)
    pub struct Analog;
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for Analog {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match *self {
                Analog => ::core::fmt::Formatter::write_str(f, "Analog"),
            }
        }
    }
    /// Push-pull output (type state)
    pub struct PushPull;
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for PushPull {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match *self {
                PushPull => ::core::fmt::Formatter::write_str(f, "PushPull"),
            }
        }
    }
    /// Open-drain output (type state)
    pub struct OpenDrain;
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for OpenDrain {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match *self {
                OpenDrain => ::core::fmt::Formatter::write_str(f, "OpenDrain"),
            }
        }
    }
    impl marker::Readable for Input {}
    impl marker::Readable for Output<OpenDrain> {}
    impl<Otype> marker::OutputSpeed for Output<Otype> {}
    impl<Otype, const AF: u8> marker::OutputSpeed for Alternate<Otype, AF> {}
    impl marker::Active for Input {}
    impl<Otype> marker::Active for Output<Otype> {}
    impl<Otype, const AF: u8> marker::Active for Alternate<Otype, AF> {}
    /// Slew rate configuration
    pub enum Speed {
        /// Low speed
        Low,
        /// Medium speed
        Medium,
        /// High speed
        High,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for Speed {}
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for Speed {
        #[inline]
        fn clone(&self) -> Speed {
            {
                *self
            }
        }
    }
    impl ::core::marker::StructuralPartialEq for Speed {}
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialEq for Speed {
        #[inline]
        fn eq(&self, other: &Speed) -> bool {
            {
                let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                if true && __self_vi == __arg_1_vi {
                    match (&*self, &*other) {
                        _ => true,
                    }
                } else {
                    false
                }
            }
        }
    }
    impl ::core::marker::StructuralEq for Speed {}
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::Eq for Speed {
        #[inline]
        #[doc(hidden)]
        #[no_coverage]
        fn assert_receiver_is_total_eq(&self) -> () {
            {}
        }
    }
    /// Internal pull-up and pull-down resistor configuration
    pub enum Resistor {
        /// Floating
        Floating,
        /// Pulled up
        PullUp,
        /// Pulled down
        PullDown,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for Resistor {}
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for Resistor {
        #[inline]
        fn clone(&self) -> Resistor {
            {
                *self
            }
        }
    }
    impl ::core::marker::StructuralPartialEq for Resistor {}
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialEq for Resistor {
        #[inline]
        fn eq(&self, other: &Resistor) -> bool {
            {
                let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                if true && __self_vi == __arg_1_vi {
                    match (&*self, &*other) {
                        _ => true,
                    }
                } else {
                    false
                }
            }
        }
    }
    impl ::core::marker::StructuralEq for Resistor {}
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::Eq for Resistor {
        #[inline]
        #[doc(hidden)]
        #[no_coverage]
        fn assert_receiver_is_total_eq(&self) -> () {
            {}
        }
    }
    /// GPIO interrupt trigger edge selection
    pub enum Edge {
        /// Rising edge of voltage
        Rising,
        /// Falling edge of voltage
        Falling,
        /// Rising and falling edge of voltage
        RisingFalling,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for Edge {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match (&*self,) {
                (&Edge::Rising,) => ::core::fmt::Formatter::write_str(f, "Rising"),
                (&Edge::Falling,) => ::core::fmt::Formatter::write_str(f, "Falling"),
                (&Edge::RisingFalling,) => ::core::fmt::Formatter::write_str(f, "RisingFalling"),
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for Edge {}
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for Edge {
        #[inline]
        fn clone(&self) -> Edge {
            {
                *self
            }
        }
    }
    impl ::core::marker::StructuralPartialEq for Edge {}
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialEq for Edge {
        #[inline]
        fn eq(&self, other: &Edge) -> bool {
            {
                let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                if true && __self_vi == __arg_1_vi {
                    match (&*self, &*other) {
                        _ => true,
                    }
                } else {
                    false
                }
            }
        }
    }
    impl ::core::marker::StructuralEq for Edge {}
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::Eq for Edge {
        #[inline]
        #[doc(hidden)]
        #[no_coverage]
        fn assert_receiver_is_total_eq(&self) -> () {
            {}
        }
    }
    /// Generic pin
    pub struct Pin<Gpio, Index, Mode> {
        pub(crate) gpio: Gpio,
        pub(crate) index: Index,
        _mode: PhantomData<Mode>,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl<Gpio: ::core::fmt::Debug, Index: ::core::fmt::Debug, Mode: ::core::fmt::Debug>
        ::core::fmt::Debug for Pin<Gpio, Index, Mode>
    {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match *self {
                Pin {
                    gpio: ref __self_0_0,
                    index: ref __self_0_1,
                    _mode: ref __self_0_2,
                } => {
                    let debug_trait_builder = &mut ::core::fmt::Formatter::debug_struct(f, "Pin");
                    let _ = ::core::fmt::DebugStruct::field(
                        debug_trait_builder,
                        "gpio",
                        &&(*__self_0_0),
                    );
                    let _ = ::core::fmt::DebugStruct::field(
                        debug_trait_builder,
                        "index",
                        &&(*__self_0_1),
                    );
                    let _ = ::core::fmt::DebugStruct::field(
                        debug_trait_builder,
                        "_mode",
                        &&(*__self_0_2),
                    );
                    ::core::fmt::DebugStruct::finish(debug_trait_builder)
                }
            }
        }
    }
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
                Speed::Medium => ospeedr.medium(self.index.index()),
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
            unsafe { (*self.gpio.ptr()).set_high(self.index.index()) };
            Ok(())
        }
        fn set_low(&mut self) -> Result<(), Self::Error> {
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
            Ok(unsafe { (*self.gpio.ptr()).is_set_low(self.index.index()) })
        }
    }
    impl<Gpio, Index, Otype> toggleable::Default for Pin<Gpio, Index, Output<Otype>>
    where
        Gpio: marker::Gpio,
        Index: marker::Index,
    {
    }
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
    ///Alternate function 0 (type state)
    pub type AF0<Otype> = Alternate<Otype, 0>;
    impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode>
    where
        Self: marker::IntoAf<0>,
        Gpio: marker::GpioStatic,
        Index: marker::Index,
    {
        /// Configures the pin to operate as an alternate function push-pull output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_push_pull()` instead"
        )]
        pub fn into_af0_push_pull(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<0>>::AFR,
        ) -> Pin<Gpio, Index, AF0<PushPull>> {
            self.into_af_push_pull::<0>(moder, otyper, afr)
        }
        /// Configures the pin to operate as an alternate function open-drain output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_open_drain()` instead"
        )]
        pub fn into_af0_open_drain(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<0>>::AFR,
        ) -> Pin<Gpio, Index, AF0<OpenDrain>> {
            self.into_af_open_drain::<0>(moder, otyper, afr)
        }
    }
    ///Alternate function 1 (type state)
    pub type AF1<Otype> = Alternate<Otype, 1>;
    impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode>
    where
        Self: marker::IntoAf<1>,
        Gpio: marker::GpioStatic,
        Index: marker::Index,
    {
        /// Configures the pin to operate as an alternate function push-pull output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_push_pull()` instead"
        )]
        pub fn into_af1_push_pull(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<1>>::AFR,
        ) -> Pin<Gpio, Index, AF1<PushPull>> {
            self.into_af_push_pull::<1>(moder, otyper, afr)
        }
        /// Configures the pin to operate as an alternate function open-drain output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_open_drain()` instead"
        )]
        pub fn into_af1_open_drain(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<1>>::AFR,
        ) -> Pin<Gpio, Index, AF1<OpenDrain>> {
            self.into_af_open_drain::<1>(moder, otyper, afr)
        }
    }
    ///Alternate function 2 (type state)
    pub type AF2<Otype> = Alternate<Otype, 2>;
    impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode>
    where
        Self: marker::IntoAf<2>,
        Gpio: marker::GpioStatic,
        Index: marker::Index,
    {
        /// Configures the pin to operate as an alternate function push-pull output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_push_pull()` instead"
        )]
        pub fn into_af2_push_pull(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<2>>::AFR,
        ) -> Pin<Gpio, Index, AF2<PushPull>> {
            self.into_af_push_pull::<2>(moder, otyper, afr)
        }
        /// Configures the pin to operate as an alternate function open-drain output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_open_drain()` instead"
        )]
        pub fn into_af2_open_drain(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<2>>::AFR,
        ) -> Pin<Gpio, Index, AF2<OpenDrain>> {
            self.into_af_open_drain::<2>(moder, otyper, afr)
        }
    }
    ///Alternate function 3 (type state)
    pub type AF3<Otype> = Alternate<Otype, 3>;
    impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode>
    where
        Self: marker::IntoAf<3>,
        Gpio: marker::GpioStatic,
        Index: marker::Index,
    {
        /// Configures the pin to operate as an alternate function push-pull output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_push_pull()` instead"
        )]
        pub fn into_af3_push_pull(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<3>>::AFR,
        ) -> Pin<Gpio, Index, AF3<PushPull>> {
            self.into_af_push_pull::<3>(moder, otyper, afr)
        }
        /// Configures the pin to operate as an alternate function open-drain output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_open_drain()` instead"
        )]
        pub fn into_af3_open_drain(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<3>>::AFR,
        ) -> Pin<Gpio, Index, AF3<OpenDrain>> {
            self.into_af_open_drain::<3>(moder, otyper, afr)
        }
    }
    ///Alternate function 4 (type state)
    pub type AF4<Otype> = Alternate<Otype, 4>;
    impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode>
    where
        Self: marker::IntoAf<4>,
        Gpio: marker::GpioStatic,
        Index: marker::Index,
    {
        /// Configures the pin to operate as an alternate function push-pull output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_push_pull()` instead"
        )]
        pub fn into_af4_push_pull(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<4>>::AFR,
        ) -> Pin<Gpio, Index, AF4<PushPull>> {
            self.into_af_push_pull::<4>(moder, otyper, afr)
        }
        /// Configures the pin to operate as an alternate function open-drain output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_open_drain()` instead"
        )]
        pub fn into_af4_open_drain(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<4>>::AFR,
        ) -> Pin<Gpio, Index, AF4<OpenDrain>> {
            self.into_af_open_drain::<4>(moder, otyper, afr)
        }
    }
    ///Alternate function 5 (type state)
    pub type AF5<Otype> = Alternate<Otype, 5>;
    impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode>
    where
        Self: marker::IntoAf<5>,
        Gpio: marker::GpioStatic,
        Index: marker::Index,
    {
        /// Configures the pin to operate as an alternate function push-pull output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_push_pull()` instead"
        )]
        pub fn into_af5_push_pull(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<5>>::AFR,
        ) -> Pin<Gpio, Index, AF5<PushPull>> {
            self.into_af_push_pull::<5>(moder, otyper, afr)
        }
        /// Configures the pin to operate as an alternate function open-drain output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_open_drain()` instead"
        )]
        pub fn into_af5_open_drain(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<5>>::AFR,
        ) -> Pin<Gpio, Index, AF5<OpenDrain>> {
            self.into_af_open_drain::<5>(moder, otyper, afr)
        }
    }
    ///Alternate function 6 (type state)
    pub type AF6<Otype> = Alternate<Otype, 6>;
    impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode>
    where
        Self: marker::IntoAf<6>,
        Gpio: marker::GpioStatic,
        Index: marker::Index,
    {
        /// Configures the pin to operate as an alternate function push-pull output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_push_pull()` instead"
        )]
        pub fn into_af6_push_pull(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<6>>::AFR,
        ) -> Pin<Gpio, Index, AF6<PushPull>> {
            self.into_af_push_pull::<6>(moder, otyper, afr)
        }
        /// Configures the pin to operate as an alternate function open-drain output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_open_drain()` instead"
        )]
        pub fn into_af6_open_drain(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<6>>::AFR,
        ) -> Pin<Gpio, Index, AF6<OpenDrain>> {
            self.into_af_open_drain::<6>(moder, otyper, afr)
        }
    }
    ///Alternate function 7 (type state)
    pub type AF7<Otype> = Alternate<Otype, 7>;
    impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode>
    where
        Self: marker::IntoAf<7>,
        Gpio: marker::GpioStatic,
        Index: marker::Index,
    {
        /// Configures the pin to operate as an alternate function push-pull output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_push_pull()` instead"
        )]
        pub fn into_af7_push_pull(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<7>>::AFR,
        ) -> Pin<Gpio, Index, AF7<PushPull>> {
            self.into_af_push_pull::<7>(moder, otyper, afr)
        }
        /// Configures the pin to operate as an alternate function open-drain output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_open_drain()` instead"
        )]
        pub fn into_af7_open_drain(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<7>>::AFR,
        ) -> Pin<Gpio, Index, AF7<OpenDrain>> {
            self.into_af_open_drain::<7>(moder, otyper, afr)
        }
    }
    ///Alternate function 8 (type state)
    pub type AF8<Otype> = Alternate<Otype, 8>;
    impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode>
    where
        Self: marker::IntoAf<8>,
        Gpio: marker::GpioStatic,
        Index: marker::Index,
    {
        /// Configures the pin to operate as an alternate function push-pull output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_push_pull()` instead"
        )]
        pub fn into_af8_push_pull(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<8>>::AFR,
        ) -> Pin<Gpio, Index, AF8<PushPull>> {
            self.into_af_push_pull::<8>(moder, otyper, afr)
        }
        /// Configures the pin to operate as an alternate function open-drain output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_open_drain()` instead"
        )]
        pub fn into_af8_open_drain(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<8>>::AFR,
        ) -> Pin<Gpio, Index, AF8<OpenDrain>> {
            self.into_af_open_drain::<8>(moder, otyper, afr)
        }
    }
    ///Alternate function 9 (type state)
    pub type AF9<Otype> = Alternate<Otype, 9>;
    impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode>
    where
        Self: marker::IntoAf<9>,
        Gpio: marker::GpioStatic,
        Index: marker::Index,
    {
        /// Configures the pin to operate as an alternate function push-pull output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_push_pull()` instead"
        )]
        pub fn into_af9_push_pull(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<9>>::AFR,
        ) -> Pin<Gpio, Index, AF9<PushPull>> {
            self.into_af_push_pull::<9>(moder, otyper, afr)
        }
        /// Configures the pin to operate as an alternate function open-drain output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_open_drain()` instead"
        )]
        pub fn into_af9_open_drain(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<9>>::AFR,
        ) -> Pin<Gpio, Index, AF9<OpenDrain>> {
            self.into_af_open_drain::<9>(moder, otyper, afr)
        }
    }
    ///Alternate function 10 (type state)
    pub type AF10<Otype> = Alternate<Otype, 10>;
    impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode>
    where
        Self: marker::IntoAf<10>,
        Gpio: marker::GpioStatic,
        Index: marker::Index,
    {
        /// Configures the pin to operate as an alternate function push-pull output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_push_pull()` instead"
        )]
        pub fn into_af10_push_pull(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<10>>::AFR,
        ) -> Pin<Gpio, Index, AF10<PushPull>> {
            self.into_af_push_pull::<10>(moder, otyper, afr)
        }
        /// Configures the pin to operate as an alternate function open-drain output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_open_drain()` instead"
        )]
        pub fn into_af10_open_drain(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<10>>::AFR,
        ) -> Pin<Gpio, Index, AF10<OpenDrain>> {
            self.into_af_open_drain::<10>(moder, otyper, afr)
        }
    }
    ///Alternate function 11 (type state)
    pub type AF11<Otype> = Alternate<Otype, 11>;
    impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode>
    where
        Self: marker::IntoAf<11>,
        Gpio: marker::GpioStatic,
        Index: marker::Index,
    {
        /// Configures the pin to operate as an alternate function push-pull output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_push_pull()` instead"
        )]
        pub fn into_af11_push_pull(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<11>>::AFR,
        ) -> Pin<Gpio, Index, AF11<PushPull>> {
            self.into_af_push_pull::<11>(moder, otyper, afr)
        }
        /// Configures the pin to operate as an alternate function open-drain output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_open_drain()` instead"
        )]
        pub fn into_af11_open_drain(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<11>>::AFR,
        ) -> Pin<Gpio, Index, AF11<OpenDrain>> {
            self.into_af_open_drain::<11>(moder, otyper, afr)
        }
    }
    ///Alternate function 12 (type state)
    pub type AF12<Otype> = Alternate<Otype, 12>;
    impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode>
    where
        Self: marker::IntoAf<12>,
        Gpio: marker::GpioStatic,
        Index: marker::Index,
    {
        /// Configures the pin to operate as an alternate function push-pull output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_push_pull()` instead"
        )]
        pub fn into_af12_push_pull(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<12>>::AFR,
        ) -> Pin<Gpio, Index, AF12<PushPull>> {
            self.into_af_push_pull::<12>(moder, otyper, afr)
        }
        /// Configures the pin to operate as an alternate function open-drain output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_open_drain()` instead"
        )]
        pub fn into_af12_open_drain(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<12>>::AFR,
        ) -> Pin<Gpio, Index, AF12<OpenDrain>> {
            self.into_af_open_drain::<12>(moder, otyper, afr)
        }
    }
    ///Alternate function 13 (type state)
    pub type AF13<Otype> = Alternate<Otype, 13>;
    impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode>
    where
        Self: marker::IntoAf<13>,
        Gpio: marker::GpioStatic,
        Index: marker::Index,
    {
        /// Configures the pin to operate as an alternate function push-pull output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_push_pull()` instead"
        )]
        pub fn into_af13_push_pull(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<13>>::AFR,
        ) -> Pin<Gpio, Index, AF13<PushPull>> {
            self.into_af_push_pull::<13>(moder, otyper, afr)
        }
        /// Configures the pin to operate as an alternate function open-drain output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_open_drain()` instead"
        )]
        pub fn into_af13_open_drain(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<13>>::AFR,
        ) -> Pin<Gpio, Index, AF13<OpenDrain>> {
            self.into_af_open_drain::<13>(moder, otyper, afr)
        }
    }
    ///Alternate function 14 (type state)
    pub type AF14<Otype> = Alternate<Otype, 14>;
    impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode>
    where
        Self: marker::IntoAf<14>,
        Gpio: marker::GpioStatic,
        Index: marker::Index,
    {
        /// Configures the pin to operate as an alternate function push-pull output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_push_pull()` instead"
        )]
        pub fn into_af14_push_pull(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<14>>::AFR,
        ) -> Pin<Gpio, Index, AF14<PushPull>> {
            self.into_af_push_pull::<14>(moder, otyper, afr)
        }
        /// Configures the pin to operate as an alternate function open-drain output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_open_drain()` instead"
        )]
        pub fn into_af14_open_drain(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<14>>::AFR,
        ) -> Pin<Gpio, Index, AF14<OpenDrain>> {
            self.into_af_open_drain::<14>(moder, otyper, afr)
        }
    }
    ///Alternate function 15 (type state)
    pub type AF15<Otype> = Alternate<Otype, 15>;
    impl<Gpio, Index, Mode> Pin<Gpio, Index, Mode>
    where
        Self: marker::IntoAf<15>,
        Gpio: marker::GpioStatic,
        Index: marker::Index,
    {
        /// Configures the pin to operate as an alternate function push-pull output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_push_pull()` instead"
        )]
        pub fn into_af15_push_pull(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<15>>::AFR,
        ) -> Pin<Gpio, Index, AF15<PushPull>> {
            self.into_af_push_pull::<15>(moder, otyper, afr)
        }
        /// Configures the pin to operate as an alternate function open-drain output pin
        #[deprecated(
            since = "0.9.0",
            note = "Will be removed with the next version. Use `into_af_open_drain()` instead"
        )]
        pub fn into_af15_open_drain(
            self,
            moder: &mut Gpio::MODER,
            otyper: &mut Gpio::OTYPER,
            afr: &mut <Self as marker::IntoAf<15>>::AFR,
        ) -> Pin<Gpio, Index, AF15<OpenDrain>> {
            self.into_af_open_drain::<15>(moder, otyper, afr)
        }
    }
}
pub mod prelude {
    pub use crate::gpio::GpioExt as _psoc6_hal_gpio_GpioExt;
    pub use embedded_hal::digital::v2::{InputPin, OutputPin};
}
pub use embedded_hal;
mod private {
    /// Private sealed trait to seal all GPIO implementations
    /// which do implement peripheral functionalities.
    pub trait Sealed {}
}
