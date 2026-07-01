//! Edge colour = the set of channels (R/G/B) an edge contributes to.
//! Port of `core/EdgeColor.h` as bitflags rather than a bare enum.

/// Which colour channels an edge belongs to. Bit 0 = red, 1 = green, 2 = blue.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum EdgeColor {
    Black = 0,
    Red = 1,
    Green = 2,
    Yellow = 3,
    Blue = 4,
    Magenta = 5,
    Cyan = 6,
    White = 7,
}

impl EdgeColor {
    #[inline]
    pub const fn bits(self) -> u8 {
        self as u8
    }

    #[inline]
    pub fn from_bits(b: u8) -> EdgeColor {
        match b & 7 {
            0 => EdgeColor::Black,
            1 => EdgeColor::Red,
            2 => EdgeColor::Green,
            3 => EdgeColor::Yellow,
            4 => EdgeColor::Blue,
            5 => EdgeColor::Magenta,
            6 => EdgeColor::Cyan,
            _ => EdgeColor::White,
        }
    }

    #[inline]
    pub fn has_red(self) -> bool {
        self.bits() & 1 != 0
    }
    #[inline]
    pub fn has_green(self) -> bool {
        self.bits() & 2 != 0
    }
    #[inline]
    pub fn has_blue(self) -> bool {
        self.bits() & 4 != 0
    }

    /// Channel bit `i` (0=R,1=G,2=B) set?
    #[inline]
    pub fn has_channel(self, i: usize) -> bool {
        self.bits() & (1 << i) != 0
    }
}

impl Default for EdgeColor {
    #[inline]
    fn default() -> Self {
        EdgeColor::White
    }
}
