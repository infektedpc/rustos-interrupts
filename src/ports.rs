//! Port I/O abstraction + mock ports for host testing.
//!
//! Same contract as `rustos-drivers::ports` (kept self-contained so this
//! crate has no sibling dependency): drivers are generic over `PortIo`,
//! host tests use `MockPorts`, a kernel uses `RealPorts`.

/// Byte-wide x86 I/O-port access.
pub trait PortIo {
    fn read_u8(&mut self, port: u16) -> u8;
    fn write_u8(&mut self, port: u16, value: u8);
    fn read_u16(&mut self, port: u16) -> u16 {
        let lo = self.read_u8(port) as u16;
        let hi = self.read_u8(port + 1) as u16;
        (hi << 8) | lo
    }
    fn write_u16(&mut self, port: u16, value: u16) {
        self.write_u8(port, value as u8);
        self.write_u8(port + 1, (value >> 8) as u8);
    }
}

/// In-memory fake ports for host tests. Covers ports 0x0000..=0x03FF,
/// which includes the PICs (0x20/0x21, 0xA0/0xA1), the PIT (0x40/0x43)
/// and the legacy I/O-wait port (0x80). Every write is appended to a log
/// so tests can assert exactly what a driver emitted.
pub struct MockPorts {
    regs: [u8; 1024],
    write_log: [(u16, u8); 1024],
    write_log_len: usize,
}

impl MockPorts {
    pub fn new() -> Self {
        Self { regs: [0; 1024], write_log: [(0, 0); 1024], write_log_len: 0 }
    }
    /// Preset a port's read value (e.g. fake a PIC ISR register).
    pub fn set(&mut self, port: u16, value: u8) {
        if (port as usize) < self.regs.len() { self.regs[port as usize] = value; }
    }
    pub fn get(&self, port: u16) -> u8 {
        if (port as usize) < self.regs.len() { self.regs[port as usize] } else { 0 }
    }
    pub fn write_log_len(&self) -> usize { self.write_log_len }
    pub fn write_log_entry(&self, i: usize) -> Option<(u16, u8)> {
        if i < self.write_log_len { Some(self.write_log[i]) } else { None }
    }
    /// Bytes written to `port`, in order.
    pub fn bytes_written_to(&self, port: u16, out: &mut [u8]) -> usize {
        let mut n = 0;
        for i in 0..self.write_log_len {
            if self.write_log[i].0 == port && n < out.len() {
                out[n] = self.write_log[i].1;
                n += 1;
            }
        }
        n
    }
    /// Did the driver write `value` to `port` at any point?
    pub fn wrote(&self, port: u16, value: u8) -> bool {
        (0..self.write_log_len).any(|i| self.write_log[i] == (port, value))
    }
    /// How many times was `value` written to `port`?
    pub fn write_count(&self, port: u16, value: u8) -> usize {
        (0..self.write_log_len).filter(|&i| self.write_log[i] == (port, value)).count()
    }
}

impl Default for MockPorts {
    fn default() -> Self { Self::new() }
}

impl PortIo for MockPorts {
    fn read_u8(&mut self, port: u16) -> u8 { self.get(port) }
    fn write_u8(&mut self, port: u16, value: u8) {
        if (port as usize) < self.regs.len() { self.regs[port as usize] = value; }
        if self.write_log_len < self.write_log.len() {
            self.write_log[self.write_log_len] = (port, value);
            self.write_log_len += 1;
        }
    }
}

/// Real x86_64 port I/O via `in`/`out`. Only on x86_64.
#[cfg(target_arch = "x86_64")]
pub struct RealPorts;

#[cfg(target_arch = "x86_64")]
impl PortIo for RealPorts {
    fn read_u8(&mut self, port: u16) -> u8 {
        let value: u8;
        unsafe { core::arch::asm!("in al, dx", in("dx") port, out("al") value, options(nomem, nostack, preserves_flags)); }
        value
    }
    fn write_u8(&mut self, port: u16, value: u8) {
        unsafe { core::arch::asm!("out dx, al", in("dx") port, in("al") value, options(nomem, nostack, preserves_flags)); }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn mock_ports_roundtrip_and_log() {
        let mut m = MockPorts::new();
        m.write_u8(0x20, 0x11);
        assert_eq!(m.read_u8(0x20), 0x11);
        assert!(m.wrote(0x20, 0x11));
        assert!(!m.wrote(0x20, 0x12));
        assert_eq!(m.write_count(0x20, 0x11), 1);
        let mut out = [0u8; 4];
        assert_eq!(m.bytes_written_to(0x20, &mut out), 1);
        m.set(0x21, 0xFF);
        assert_eq!(m.read_u8(0x21), 0xFF);
    }
}
