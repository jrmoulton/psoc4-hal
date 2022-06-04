// #![warn(missing_docs)]
// #![deny(warnings)]
#![no_std]

pub use psoc4_pac as pac;

pub mod gpio;
pub mod prelude;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
