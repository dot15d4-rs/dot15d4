#![allow(dead_code)]
/// The absolute slot number represents the total number of timeslots that has
/// elapsed since the start of the network or an arbitrary start time
/// determined by the PAN coordinator. It is stored as a 5-byte unsigned
/// integer.
#[derive(Default, Clone, Copy)]
pub struct AbsoluteSlotNumber {
    /// least significant 4 bytes of the absolute slot number
    ls4b: u32,
    /// most significant byte of the absolute slot number
    ms1b: u8,
}

impl AbsoluteSlotNumber {
    /// Increments the ASN by one slot
    pub fn increment(&mut self) {
        let ls4b = self.ls4b;
        self.ls4b += 1;
        if ls4b > self.ls4b {
            self.ms1b += 1;
        }
    }
    /// Decrements the ASN by one slot
    pub fn decrement(&mut self) {
        let ls4b = self.ls4b;
        self.ls4b -= 1;
        if ls4b < self.ls4b {
            self.ms1b -= 1;
        }
    }
}

impl PartialEq for AbsoluteSlotNumber {
    fn eq(&self, other: &Self) -> bool {
        self.ls4b == other.ls4b && self.ms1b == other.ms1b
    }
}

impl PartialEq<i64> for AbsoluteSlotNumber {
    fn eq(&self, other: &i64) -> bool {
        match Self::try_from(*other) {
            Ok(asn) => asn == *self,
            Err(_) => false,
        }
    }
}

impl PartialOrd for AbsoluteSlotNumber {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        if self.ms1b < other.ms1b || self.ls4b < other.ls4b {
            Some(core::cmp::Ordering::Less)
        } else if self.ls4b > other.ls4b {
            Some(core::cmp::Ordering::Greater)
        } else {
            Some(core::cmp::Ordering::Equal)
        }
    }
}

impl core::ops::Add<u32> for AbsoluteSlotNumber {
    type Output = Self;

    fn add(self, rhs: u32) -> Self::Output {
        let ls4b = self.ls4b + rhs;
        let mut ms1b = self.ms1b;
        if ls4b < self.ls4b {
            ms1b += 1;
        }
        Self { ls4b, ms1b }
    }
}

impl core::ops::Add<u16> for AbsoluteSlotNumber {
    type Output = Self;

    fn add(self, rhs: u16) -> Self::Output {
        let ls4b = self.ls4b + rhs as u32;
        let mut ms1b = self.ms1b;
        if ls4b < self.ls4b {
            ms1b += 1;
        }
        Self { ls4b, ms1b }
    }
}

impl core::ops::Add<i32> for AbsoluteSlotNumber {
    type Output = Self;

    fn add(self, rhs: i32) -> Self::Output {
        let ls4b = self.ls4b + rhs as u32;
        let mut ms1b = self.ms1b;
        if ls4b < self.ls4b {
            ms1b += 1;
        }
        Self { ls4b, ms1b }
    }
}

impl core::ops::Sub<u32> for AbsoluteSlotNumber {
    type Output = Self;

    fn sub(self, rhs: u32) -> Self::Output {
        let ls4b = self.ls4b - rhs;
        let mut ms1b = self.ms1b;
        if ls4b > self.ls4b {
            ms1b -= 1;
        }
        Self { ls4b, ms1b }
    }
}

impl core::ops::Sub<AbsoluteSlotNumber> for AbsoluteSlotNumber {
    type Output = u32;

    fn sub(self, rhs: AbsoluteSlotNumber) -> Self::Output {
        self.ls4b - rhs.ls4b
    }
}

impl core::ops::Rem<u16> for AbsoluteSlotNumber {
    type Output = u16;

    fn rem(self, rhs: u16) -> u16 {
        let remainder = ((0xffffffff % (rhs as u32)) + 1) % (rhs as u32);
        let mut result = self.ls4b % rhs as u32;
        result += self.ms1b as u32 * remainder % rhs as u32;
        result as u16
    }
}

impl TryFrom<i64> for AbsoluteSlotNumber {
    type Error = ();

    fn try_from(value: i64) -> Result<Self, Self::Error> {
        const MAX_VALUE: i64 = 0xffffffffff;
        if !(0..=MAX_VALUE).contains(&value) {
            return Err(());
        }
        Ok(Self {
            ls4b: (value & 0xffffffff) as u32,
            ms1b: ((value & 0xff00000000) >> 32) as u8,
        })
    }
}

impl TryFrom<i32> for AbsoluteSlotNumber {
    type Error = ();

    fn try_from(value: i32) -> Result<Self, Self::Error> {
        if value < 0 {
            Err(())
        } else {
            Ok(Self {
                ls4b: value as u32,
                ms1b: 0,
            })
        }
    }
}

impl TryFrom<u32> for AbsoluteSlotNumber {
    type Error = ();

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        Ok(Self {
            ls4b: value,
            ms1b: 0,
        })
    }
}

impl TryInto<i64> for AbsoluteSlotNumber {
    type Error = ();

    fn try_into(self) -> Result<i64, Self::Error> {
        Ok(self.ls4b as i64 + ((self.ms1b as i64) << 32))
    }
}

impl TryInto<u32> for AbsoluteSlotNumber {
    type Error = ();

    fn try_into(self) -> Result<u32, Self::Error> {
        match self.ms1b {
            0 => Ok(self.ls4b),
            _ => Err(()),
        }
    }
}

#[cfg(test)]
pub mod tests {
    const MAX_VALUE: i64 = 0xffffffffff;
    use super::AbsoluteSlotNumber;

    #[test]
    fn asn_conversions() {
        let asn: AbsoluteSlotNumber = AbsoluteSlotNumber::try_from(0xAB12345678_i64).unwrap();
        assert_eq!(asn.ms1b, 0xab);
        assert_eq!(asn.ls4b, 0x12345678);
        let asn: AbsoluteSlotNumber = AbsoluteSlotNumber::try_from(MAX_VALUE).unwrap();
        assert_eq!(asn.ms1b, 0xff);
        assert_eq!(asn.ls4b, 0xffffffff);

        // Invalid values
        let asn = AbsoluteSlotNumber::try_from(MAX_VALUE + 1);
        assert!(asn.is_err());
        let asn = AbsoluteSlotNumber::try_from(-1);
        assert!(asn.is_err());
    }
    #[test]
    fn asn_operations() {
        let mut asn1: AbsoluteSlotNumber = AbsoluteSlotNumber::try_from(42).unwrap();
        let asn2: AbsoluteSlotNumber = AbsoluteSlotNumber::try_from(4242).unwrap();
        let asn3: AbsoluteSlotNumber = AbsoluteSlotNumber::try_from(4284).unwrap();
        assert!(asn1 < asn2);
        assert!(asn2 + 42 == asn3);
        assert!(asn2 - 42 == 4200);
        assert!(asn2 - asn1 == 4200);
        asn1.increment();
        assert!(asn1 == 43);
        asn1.decrement();
        assert!(asn1 == 42);
    }
}
