// #![warn(missing_docs)]
// #![deny(warnings)]
#![no_std]

pub use psoc4_pac as pac;

pub mod gpio;
pub mod prelude;
pub use embedded_hal;
pub(crate) use embedded_hal as hal;

mod private {
    /// Private sealed trait to seal all GPIO implementations
    /// which do implement peripheral functionalities.
    pub trait Sealed {}

    // Modify specific index of array-like register
    // macro_rules! modify_at {
    //     ($reg:expr, $bitwidth:expr, $index:expr, $value:expr) => {
    //         $reg.modify(|r, w| {
    //             let mask = !(u32::MAX >> (32 - $bitwidth) << ($bitwidth * $index));
    //             let value = $value << ($bitwidth * $index);
    //             w.bits(r.bits() & mask | value)
    //         })
    //     };
    // }
    // pub(crate) use modify_at;
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
