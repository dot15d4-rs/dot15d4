use dot15d4::{driver::socs::nrf::export::pac::RNG, RngCore, RngError};

pub struct NodeRng(u64);

impl NodeRng {
    pub fn new(seed: u64) -> Self {
        Self(seed)
    }

    pub fn from_trng(rng: RNG) -> Self {
        rng.config.write(|w| w.dercen().enabled());
        rng.tasks_start.write(|w| w.tasks_start().set_bit());

        let mut seed = 0u64;
        for _ in 0..8 {
            while rng.events_valrdy.read().events_valrdy().bit_is_clear() {}
            rng.events_valrdy.reset();
            seed = (seed << 8) | rng.value.read().value().bits() as u64;
        }

        rng.tasks_stop.write(|w| w.tasks_stop().set_bit());

        Self::new(seed)
    }
}

impl RngCore for NodeRng {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn next_u32(&mut self) -> u32 {
        self.next_u64() as u32
    }
    fn fill_bytes(&mut self, dest: &mut [u8]) {
        for chunk in dest.chunks_mut(8) {
            let bytes = self.next_u64().to_le_bytes();
            chunk.copy_from_slice(&bytes[..chunk.len()]);
        }
    }
    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), RngError> {
        self.fill_bytes(dest);
        Ok(())
    }
}
