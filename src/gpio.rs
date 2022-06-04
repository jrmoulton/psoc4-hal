//! General purpose input / output
//!
//! Taken in majority from https://github.com/psoc-rs/psoc6-hal but changes have been made
//! MIT/APACHE2 License

use core::marker::PhantomData;

/// Extension trait to split a GPIO peripheral in independent pins and
/// registers.
pub trait GpioExt {
    /// The parts to split the GPIO into.
    type Parts;

    /// Splits the GPIO block into independent pins and registers.
    fn split(self) -> Self::Parts;
}

/// HSIOM GPIO mode (type state)
pub struct GpioPinMode;

/// High impedance drive mode (type state)
pub struct HighZ;

/// Strong output drive mode
pub struct Strong;

/// Input mode (type state)
pub struct Input<Mode> {
    _mode: PhantomData<Mode>,
}

/// Output mode (type state)
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}

// `i` -> port number
// `j` -> pin number
macro_rules! gpio {
    ([
     $($Pi_j:ident: ($pi_j:ident, $prti:ident, $j:expr, $Mode:ty)),+ $(,)*
    ]) => {
        use core::convert::Infallible;

        use embedded_hal::digital::v2::OutputPin;
        use psoc4_pac::GPIO;

        /// GPIO parts
        pub struct Parts {
            $(
                /// Pin
                pub $pi_j: $Pi_j<$Mode>,
            )+
        }

        impl GpioExt for GPIO {
            type Parts = Parts;

            fn split(self) -> Parts {
                Parts {
                    $(
                        $pi_j: $Pi_j { _mode: PhantomData },
                    )+
                }
            }
        }

        $(
            /// Pin
            pub struct $Pi_j<MODE> {
                _mode: PhantomData<MODE>,
            }

            impl<MODE> $Pi_j<MODE> {
                /// Configures the pin to operate as a strong output pin
                pub fn into_strong_output(self) -> $Pi_j<Output<Strong>> {
                    self.set_drive_mode(6);
                    $Pi_j { _mode: PhantomData }
                }

                /// Set the drive mode for the pin
                fn set_drive_mode(&self, bits: u8) {
                    unsafe { (*GPIO::ptr()).$prti.pc.modify(|_, w| {
                        match $j {
                            0 => w.dm0().bits(bits),
                            1 => w.dm1().bits(bits),
                            2 => w.dm2().bits(bits),
                            3 => w.dm3().bits(bits),
                            4 => w.dm4().bits(bits),
                            5 => w.dm5().bits(bits),
                            6 => w.dm6().bits(bits),
                            7 => w.dm7().bits(bits),
                            _ => panic!(), //psoc only support up to 8 pins per port
                        }
                    })}
                }
            }

            impl<Mode> OutputPin for $Pi_j<Output<Mode>> {
                type Error = Infallible;

                fn set_high(&mut self) -> Result<(), Self::Error> {
                    unsafe { (*GPIO::ptr()).$prti.dr_set.write(|w| w.bits(1 << $j)) };
                    Ok(())
                }

                fn set_low(&mut self) -> Result<(), Self::Error> {
                    unsafe { (*GPIO::ptr()).$prti.dr_clr.write(|w| w.bits(1 << $j)) };
                    Ok(())
                }
            }
        )+
    };
}

gpio!([
    P0_0: (p0_0, prt0, 0, Input<HighZ>),
    P0_1: (p0_1, prt0, 1, Input<HighZ>),
    P0_2: (p0_2, prt0, 2, Input<HighZ>),
    P0_3: (p0_3, prt0, 3, Input<HighZ>),
    P0_4: (p0_4, prt0, 4, Input<HighZ>),
    P0_5: (p0_5, prt0, 5, Input<HighZ>),
    P0_6: (p0_6, prt0, 6, Input<HighZ>),
    P0_7: (p0_7, prt0, 7, Input<HighZ>),

    P1_0: (p1_0, prt1, 0, Input<HighZ>),
    P1_1: (p1_1, prt1, 1, Input<HighZ>),
    P1_2: (p1_2, prt1, 2, Input<HighZ>),
    P1_3: (p1_3, prt1, 3, Input<HighZ>),
    P1_4: (p1_4, prt1, 4, Input<HighZ>),
    P1_5: (p1_5, prt1, 5, Input<HighZ>),
    P1_6: (p1_6, prt1, 6, Input<HighZ>),
    P1_7: (p1_7, prt1, 7, Input<HighZ>),

    P2_0: (p2_0, prt2, 0, Input<HighZ>),
    P2_1: (p2_1, prt2, 1, Input<HighZ>),
    P2_2: (p2_2, prt2, 2, Input<HighZ>),
    P2_3: (p2_3, prt2, 3, Input<HighZ>),
    P2_4: (p2_4, prt2, 4, Input<HighZ>),
    P2_5: (p2_5, prt2, 5, Input<HighZ>),
    P2_6: (p2_6, prt2, 6, Input<HighZ>),
    P2_7: (p2_7, prt2, 7, Input<HighZ>),

    P3_0: (p3_0, prt3, 0, Input<HighZ>),
    P3_1: (p3_1, prt3, 1, Input<HighZ>),
    P3_2: (p3_2, prt3, 2, Input<HighZ>),
    P3_3: (p3_3, prt3, 3, Input<HighZ>),
    P3_4: (p3_4, prt3, 4, Input<HighZ>),
    P3_5: (p3_5, prt3, 5, Input<HighZ>),
    P3_6: (p3_6, prt3, 6, Input<HighZ>),
    P3_7: (p3_7, prt3, 7, Input<HighZ>),

    P4_0: (p4_0, prt4, 0, Input<HighZ>),
    P4_1: (p4_1, prt4, 1, Input<HighZ>),

    // TODO: These are available on the 4200-ble but the pac doesn't know about them...
    // P5_0: (p5_0, prt5, 0, Input<HighZ>),
    // P5_1: (p5_1, prt5, 1, Input<HighZ>)
]);
