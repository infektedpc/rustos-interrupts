//! 8259 Programmable Interrupt Controller (PIC), master + slave.
//!
//! At reset the PICs route IRQs to CPU vectors 0–15, which collide with
//! CPU exceptions 0–15. `Pic::remap` moves them to vectors 32–47
//! (master 32–39, slave 40–47) with the standard ICW1–ICW4 sequence
//! (OSDev "8259 PIC"). Until remapping, no hardware IRQ can be enabled
//! — this is the step `rustos-kernel` was waiting on: its IDT only
//! serves CPU exceptions until the PICs are remapped.
//!
//! Spurious IRQs: IRQ7 (master) and IRQ15 (slave) can fire with no
//! device behind them. A spurious IRQ7 needs *no* EOI; a spurious IRQ15
//! needs an EOI to the master only. Detect them from the In-Service
//! Register (OCW3 `0x0B`, then read the command port): if the IRQ's ISR
//! bit is clear, the interrupt was spurious. See `is_spurious_irq` and
//! `Pic::send_eoi_checked`.

use crate::ports::PortIo;

/// Master PIC command / data ports.
pub const PIC1_COMMAND: u16 = 0x20;
pub const PIC1_DATA: u16 = 0x21;
/// Slave PIC command / data ports.
pub const PIC2_COMMAND: u16 = 0xA0;
pub const PIC2_DATA: u16 = 0xA1;

/// End-of-interrupt command byte.
pub const EOI: u8 = 0x20;
/// ICW1: initialise, expect ICW4.
pub const ICW1_INIT: u8 = 0x10;
pub const ICW1_ICW4: u8 = 0x01;
/// ICW4: 8086/88 mode.
pub const ICW4_8086: u8 = 0x01;
/// OCW3: read the In-Service Register on the next read.
pub const OCW3_READ_ISR: u8 = 0x0B;

/// Default vector bases after remapping (OSDev convention).
pub const PIC1_OFFSET: u8 = 32;
pub const PIC2_OFFSET: u8 = 40;

/// Port used for the classic I/O-wait delay (a write to 0x80).
pub const IO_WAIT_PORT: u16 = 0x80;

/// IRQ numbers on the two PICs.
pub const IRQ_TIMER: u8 = 0;
pub const IRQ_KEYBOARD: u8 = 1;
pub const IRQ_CASCADE: u8 = 2;
pub const IRQ_COM2: u8 = 3;
pub const IRQ_COM1: u8 = 4;
pub const IRQ_LPT2: u8 = 5;
pub const IRQ_FLOPPY: u8 = 6;
pub const IRQ_SPURIOUS_MASTER: u8 = 7;
pub const IRQ_RTC: u8 = 8;
pub const IRQ_SPURIOUS_SLAVE: u8 = 15;

/// Vector for a PIC IRQ once remapped (master base 32, slave base 40).
pub const fn irq_vector(irq: u8) -> u8 {
    if irq < 8 { PIC1_OFFSET + irq } else { PIC2_OFFSET + (irq - 8) }
}

/// True when the IRQ's bit is *clear* in the ISR byte — i.e. the
/// interrupt was spurious (no device is actually in service).
pub const fn is_spurious_irq(irq: u8, isr: u8) -> bool {
    isr & (1 << (irq % 8)) == 0
}

/// The 8259 pair, generic over port I/O so host tests can use `MockPorts`.
pub struct Pic<P: PortIo> {
    ports: P,
    offset1: u8,
    offset2: u8,
    mask1: u8,
    mask2: u8,
}

impl<P: PortIo> Pic<P> {
    pub fn new(ports: P) -> Self {
        Self { ports, offset1: PIC1_OFFSET, offset2: PIC2_OFFSET, mask1: 0xFF, mask2: 0xFF }
    }

    pub fn offset1(&self) -> u8 { self.offset1 }
    pub fn offset2(&self) -> u8 { self.offset2 }
    /// Mutable access to the underlying port I/O (e.g. for tests to
    /// inspect the write log).
    pub fn ports(&mut self) -> &mut P { &mut self.ports }
    /// Cached interrupt masks (1 = masked). Written to the data ports.
    pub fn mask1(&self) -> u8 { self.mask1 }
    pub fn mask2(&self) -> u8 { self.mask2 }

    fn io_wait(&mut self) {
        self.ports.write_u8(IO_WAIT_PORT, 0);
    }

    /// Remap the PICs to `offset1` (master) / `offset2` (slave) with the
    /// ICW1–ICW4 sequence. Preserves the current masks across the remap.
    /// Call once, before enabling any hardware interrupt.
    pub fn remap(&mut self, offset1: u8, offset2: u8) {
        self.offset1 = offset1;
        self.offset2 = offset2;
        // Save the current masks; the remap sequence clears them.
        let saved1 = self.ports.read_u8(PIC1_DATA);
        let saved2 = self.ports.read_u8(PIC2_DATA);
        self.mask1 = saved1;
        self.mask2 = saved2;

        self.ports.write_u8(PIC1_COMMAND, ICW1_INIT | ICW1_ICW4);
        self.io_wait();
        self.ports.write_u8(PIC2_COMMAND, ICW1_INIT | ICW1_ICW4);
        self.io_wait();
        self.ports.write_u8(PIC1_DATA, offset1);
        self.io_wait();
        self.ports.write_u8(PIC2_DATA, offset2);
        self.io_wait();
        self.ports.write_u8(PIC1_DATA, 0x04); // master: slave on IRQ2
        self.io_wait();
        self.ports.write_u8(PIC2_DATA, 0x02); // slave: cascade identity 2
        self.io_wait();
        self.ports.write_u8(PIC1_DATA, ICW4_8086);
        self.io_wait();
        self.ports.write_u8(PIC2_DATA, ICW4_8086);
        self.io_wait();
        // Restore masks.
        self.ports.write_u8(PIC1_DATA, self.mask1);
        self.ports.write_u8(PIC2_DATA, self.mask2);
    }

    /// Remap to the conventional bases: master 32, slave 40.
    pub fn remap_default(&mut self) {
        self.remap(PIC1_OFFSET, PIC2_OFFSET);
    }

    fn set_mask(&mut self, irq: u8, masked: bool) {
        if irq < 8 {
            if masked { self.mask1 |= 1 << irq; } else { self.mask1 &= !(1 << irq); }
            self.ports.write_u8(PIC1_DATA, self.mask1);
        } else {
            let bit = irq - 8;
            if masked { self.mask2 |= 1 << bit; } else { self.mask2 &= !(1 << bit); }
            self.ports.write_u8(PIC2_DATA, self.mask2);
        }
    }

    /// Mask (disable) an IRQ line.
    pub fn mask_irq(&mut self, irq: u8) { self.set_mask(irq, true); }
    /// Unmask (enable) an IRQ line.
    pub fn unmask_irq(&mut self, irq: u8) { self.set_mask(irq, false); }
    /// True when the IRQ line is currently masked.
    pub fn is_masked(&self, irq: u8) -> bool {
        if irq < 8 { self.mask1 & (1 << irq) != 0 } else { self.mask2 & (1 << (irq - 8)) != 0 }
    }

    /// Mask every IRQ on both PICs.
    pub fn mask_all(&mut self) {
        self.mask1 = 0xFF;
        self.mask2 = 0xFF;
        self.ports.write_u8(PIC1_DATA, 0xFF);
        self.ports.write_u8(PIC2_DATA, 0xFF);
    }

    /// Read the In-Service Register of one PIC (0 = master, 1 = slave).
    /// Issues OCW3 `0x0B`, then reads the command port.
    pub fn read_isr(&mut self, slave: bool) -> u8 {
        let cmd = if slave { PIC2_COMMAND } else { PIC1_COMMAND };
        self.ports.write_u8(cmd, OCW3_READ_ISR);
        self.ports.read_u8(cmd)
    }

    /// Send end-of-interrupt for a (non-spurious) IRQ. IRQ >= 8 needs
    /// an EOI to the slave first, then the master.
    pub fn send_eoi(&mut self, irq: u8) {
        if irq >= 8 {
            self.ports.write_u8(PIC2_COMMAND, EOI);
        }
        self.ports.write_u8(PIC1_COMMAND, EOI);
    }

    /// EOI with spurious-IRQ handling. `isr_master` / `isr_slave` are the
    /// ISR bytes read (via `read_isr`) by the caller.
    /// * Spurious IRQ7: send nothing.
    /// * Spurious IRQ15: EOI to the master only.
    /// * Anything else: normal `send_eoi`.
    pub fn send_eoi_checked(&mut self, irq: u8, isr_master: u8, isr_slave: u8) {
        match irq {
            IRQ_SPURIOUS_MASTER if is_spurious_irq(irq, isr_master) => {}
            IRQ_SPURIOUS_SLAVE if is_spurious_irq(irq, isr_slave) => {
                self.ports.write_u8(PIC1_COMMAND, EOI);
            }
            _ => self.send_eoi(irq),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ports::MockPorts;

    #[test]
    fn irq_vectors() {
        assert_eq!(irq_vector(0), 32); // timer
        assert_eq!(irq_vector(1), 33); // keyboard
        assert_eq!(irq_vector(7), 39); // spurious master
        assert_eq!(irq_vector(8), 40); // RTC
        assert_eq!(irq_vector(15), 47); // spurious slave
    }

    #[test]
    fn remap_writes_icw_sequence() {
        let mut ports = MockPorts::new();
        // Hardware masks at boot: everything masked.
        ports.set(PIC1_DATA, 0xFF);
        ports.set(PIC2_DATA, 0xFF);
        let mut pic = Pic::new(ports);
        pic.remap_default();
        // Reconstruct the ICW writes per port from the log, in order.
        let mut cmd1 = [0u8; 8];
        let n1 = pic.ports.bytes_written_to(PIC1_COMMAND, &mut cmd1);
        assert_eq!(&cmd1[..n1], &[ICW1_INIT | ICW1_ICW4]);
        let mut cmd2 = [0u8; 8];
        let n2 = pic.ports.bytes_written_to(PIC2_COMMAND, &mut cmd2);
        assert_eq!(&cmd2[..n2], &[ICW1_INIT | ICW1_ICW4]);
        // Data ports: ICW2, ICW3, ICW4, then the restored mask (0xFF).
        let mut d1 = [0u8; 8];
        let m1 = pic.ports.bytes_written_to(PIC1_DATA, &mut d1);
        assert_eq!(&d1[..m1], &[32, 0x04, ICW4_8086, 0xFF]);
        let mut d2 = [0u8; 8];
        let m2 = pic.ports.bytes_written_to(PIC2_DATA, &mut d2);
        assert_eq!(&d2[..m2], &[40, 0x02, ICW4_8086, 0xFF]);
        assert_eq!(pic.offset1(), 32);
        assert_eq!(pic.offset2(), 40);
    }

    #[test]
    fn remap_preserves_existing_masks() {
        let mut ports = MockPorts::new();
        ports.set(PIC1_DATA, 0xFC); // timer + keyboard unmasked
        ports.set(PIC2_DATA, 0xFF);
        let mut pic = Pic::new(ports);
        pic.remap_default();
        assert_eq!(pic.mask1(), 0xFC);
        assert_eq!(pic.mask2(), 0xFF);
        assert_eq!(pic.ports.get(PIC1_DATA), 0xFC);
    }

    #[test]
    fn mask_and_unmask() {
        let mut pic = Pic::new(MockPorts::new());
        assert!(pic.is_masked(IRQ_TIMER));
        pic.unmask_irq(IRQ_TIMER);
        assert!(!pic.is_masked(IRQ_TIMER));
        assert_eq!(pic.mask1(), 0xFE);
        assert!(pic.ports.wrote(PIC1_DATA, 0xFE));
        pic.unmask_irq(IRQ_KEYBOARD);
        assert_eq!(pic.mask1(), 0xFC);
        pic.mask_irq(IRQ_TIMER);
        assert!(pic.is_masked(IRQ_TIMER));
        assert_eq!(pic.mask1(), 0xFD);
        // Slave side.
        pic.unmask_irq(IRQ_RTC);
        assert!(!pic.is_masked(IRQ_RTC));
        assert_eq!(pic.mask2(), 0xFE);
        pic.mask_all();
        assert_eq!(pic.mask1(), 0xFF);
        assert_eq!(pic.mask2(), 0xFF);
    }

    #[test]
    fn eoi_routing() {
        let mut pic = Pic::new(MockPorts::new());
        pic.send_eoi(IRQ_TIMER);
        assert_eq!(pic.ports.write_count(PIC1_COMMAND, EOI), 1);
        assert_eq!(pic.ports.write_count(PIC2_COMMAND, EOI), 0);
        let mut pic = Pic::new(MockPorts::new());
        pic.send_eoi(IRQ_RTC); // slave IRQ: slave EOI first, then master
        assert_eq!(pic.ports.write_count(PIC2_COMMAND, EOI), 1);
        assert_eq!(pic.ports.write_count(PIC1_COMMAND, EOI), 1);
        let log0 = pic.ports.write_log_entry(0).unwrap();
        let log1 = pic.ports.write_log_entry(1).unwrap();
        assert_eq!(log0, (PIC2_COMMAND, EOI));
        assert_eq!(log1, (PIC1_COMMAND, EOI));
    }

    #[test]
    fn spurious_detection_and_checked_eoi() {
        // ISR bit clear -> spurious; bit set -> real.
        assert!(is_spurious_irq(7, 0x00));
        assert!(!is_spurious_irq(7, 0x80));
        assert!(is_spurious_irq(15, 0x7F));
        assert!(!is_spurious_irq(15, 0x80));

        // Spurious IRQ7: no EOI at all.
        let mut pic = Pic::new(MockPorts::new());
        pic.send_eoi_checked(7, 0x00, 0x00);
        assert_eq!(pic.ports.write_log_len(), 0);
        // Real IRQ7: normal EOI to master.
        let mut pic = Pic::new(MockPorts::new());
        pic.send_eoi_checked(7, 0x80, 0x00);
        assert_eq!(pic.ports.write_count(PIC1_COMMAND, EOI), 1);
        // Spurious IRQ15: EOI to master only, none to slave.
        let mut pic = Pic::new(MockPorts::new());
        pic.send_eoi_checked(15, 0x00, 0x00);
        assert_eq!(pic.ports.write_count(PIC1_COMMAND, EOI), 1);
        assert_eq!(pic.ports.write_count(PIC2_COMMAND, EOI), 0);
        // Real IRQ15: both.
        let mut pic = Pic::new(MockPorts::new());
        pic.send_eoi_checked(15, 0x00, 0x80);
        assert_eq!(pic.ports.write_count(PIC1_COMMAND, EOI), 1);
        assert_eq!(pic.ports.write_count(PIC2_COMMAND, EOI), 1);
    }

    #[test]
    fn read_isr_issues_ocw3() {
        let mut pic = Pic::new(MockPorts::new());
        let _ = pic.read_isr(false);
        assert!(pic.ports.wrote(PIC1_COMMAND, OCW3_READ_ISR));
        let _ = pic.read_isr(true);
        assert!(pic.ports.wrote(PIC2_COMMAND, OCW3_READ_ISR));
    }
}
