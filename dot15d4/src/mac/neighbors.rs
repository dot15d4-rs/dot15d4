#![allow(dead_code)]

use crate::driver::time::{Instant, Microseconds};

pub enum TableError {
    Full,
}

/// This trait needs to be implemented by systems using dot15d4 in order for
/// dot15d4 to maintain the neighbor table.
pub trait NeighborsTable<N, A>
where
    N: MacNeighbor,
    A: AsRef<[u8]>,
{
    fn add_neighbor(&mut self, address: A) -> Result<&N, TableError>;
    fn get_neighbor(&self, address: A) -> Option<&N>;
    fn remove_neighbor(&mut self, address: A);
}

/// Trait that allows for interaction with the concept of MAC neighbor used in
/// another crate.
pub trait MacNeighbor {
    /// The IEEE 8012.15.4 address of the neighbor.
    fn address(&self) -> [u8; 8];
    /// Time of last transmission of a packet by the neighbor.
    fn last_tx(&self) -> Instant<Microseconds>;
    /// Estimated transmission count when communicating with the neighbor.
    fn etx(&self) -> u32;
    /// Link quality indicator
    fn lqi(&self) -> u32;
    /// Number of transmissions
    fn num_tx(&self) -> u32;
    /// Number of packets received
    fn num_rx(&self) -> u32;
    /// Set the time of last transmission of a packet by the neighbor.
    fn set_last_tx(&mut self, instant: Instant<Microseconds>);
    /// Set the Estimated transmission count.
    fn set_etx(&mut self, etx: u32);
    /// Set the link quality indicator.
    fn set_lqi(&mut self, lqi: u32);
    /// Set the number of transmissions
    fn set_num_tx(&mut self, num_tx: u32);
    /// Set the number of received packets
    fn set_num_rx(&mut self, num_rx: u32);
}

#[cfg(test)]
pub mod tests {
    use crate::driver::time::{Instant, Microseconds};

    use super::MacNeighbor;
    pub(crate) struct TestNeighbor {
        address: [u8; 8],
        last_tx: Instant<Microseconds>,
        etx: u32,
        n_tx: u32,
        n_rx: u32,
        lqi: u32,
    }

    impl Default for TestNeighbor {
        fn default() -> Self {
            Self {
                address: [0; 8],
                last_tx: Instant::new(0),
                etx: 2,
                n_tx: 0,
                n_rx: 0,
                lqi: 0,
            }
        }
    }

    impl TestNeighbor {
        pub(crate) fn new(address: [u8; 8]) -> Self {
            Self {
                address,
                ..Default::default()
            }
        }
    }

    impl MacNeighbor for TestNeighbor {
        fn address(&self) -> [u8; 8] {
            self.address
        }

        fn last_tx(&self) -> Instant<Microseconds> {
            self.last_tx
        }

        fn etx(&self) -> u32 {
            self.etx
        }

        fn lqi(&self) -> u32 {
            self.lqi
        }

        fn num_tx(&self) -> u32 {
            self.n_tx
        }

        fn num_rx(&self) -> u32 {
            self.n_rx
        }

        fn set_last_tx(&mut self, instant: Instant<Microseconds>) {
            self.last_tx = instant;
        }

        fn set_etx(&mut self, etx: u32) {
            self.etx = etx;
        }

        fn set_lqi(&mut self, lqi: u32) {
            self.lqi = lqi;
        }

        fn set_num_tx(&mut self, num_tx: u32) {
            self.n_tx = num_tx;
        }

        fn set_num_rx(&mut self, num_rx: u32) {
            self.n_rx = num_rx;
        }
    }

    // pub(crate) struct TestNeighborTable<const N: usize> {
    //     neighbors: heapless::Vec<TestNeighbor, N>,
    // }

    // impl<const N: usize> TestNeighborTable<N> {
    //     pub fn new() -> Self {
    //         Self {
    //             neighbors: heapless::Vec::new(),
    //         }
    //     }
    // }

    // impl<const N: usize> NeighborsTable<TestNeighbor, [u8; 8]> for TestNeighborTable<N> {
    //     fn add_neighbor(&mut self, address: [u8; 8]) -> Result<&TestNeighbor, TableError> {
    //         if let Some(neighbor) = self.neighbors.iter().find(|nbr| nbr.address == address) {
    //             return Ok(neighbor);
    //         }
    //         let neighbor = TestNeighbor::new(address);
    //         match self.neighbors.push(neighbor) {
    //             Ok(_) => Ok(self
    //                 .neighbors
    //                 .iter()
    //                 .find(|nbr| nbr.address == address)
    //                 .unwrap()),
    //             Err(_) => Err(TableError::Full),
    //         }
    //     }

    //     fn get_neighbor(&self, address: [u8; 8]) -> Option<&TestNeighbor> {
    //         self.neighbors.iter().find(|nbr| nbr.address == address)
    //     }

    //     fn remove_neighbor(&mut self, address: [u8; 8]) {
    //         todo!()
    //     }
    // }

    // #[test]
    // fn test1() {
    //     let table = TestNeighborTable::<3>::new();
    //     let nbr1 = TestNeighbor::new([0, 0, 0, 0, 0, 0, 0, 1]);
    //     let nbr2 = TestNeighbor::new([0, 0, 0, 0, 0, 0, 0, 2]);
    //     let nbr3 = TestNeighbor::new([0, 0, 0, 0, 0, 0, 0, 3]);
    //     //
    // }
}
