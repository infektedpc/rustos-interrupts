//! Local APIC and IOAPIC.
//!
//! Register layout only — no real APIC is touched on the host. All
//! access goes through [`RegWindow`], so tests use [`MockRegWindow`]
//! (in-memory) and a kernel uses [`VolatileRegWindow`] over the
//! memory-mapped base address.
//!
//! * Local APIC base: `0xFEE00000` (from the IA32_APIC_BASE MSR; the
//!   default after reset). Registers are 32-bit, 16-byte spaced; the
//!   offsets below are byte offsets into the 4 KiB page.
//! * IOAPIC base: `0xFEC00000`. Access is indirect: write the register
//!   index to IOREGSEL, then read/write IOWIN.
//!
//! Not done here: reading the APIC base from the MSR, ACPI/MADT
//! discovery of the real bases, IOAPIC redirection-table programming
//! on live hardware (the entry builder is provided and host-tested,
//! but no kernel wires it to a real IOAPIC yet), MSI/MSI-X.

/// 32-bit register window (byte offsets), e.g. an APIC MMIO page.
pub trait RegWindow {
    fn read(&mut self, offset: u32) -> u32;
    fn write(&mut self, offset: u32, value: u32);
}

/// In-memory register window for host tests.
#[derive(Clone)]
pub struct MockRegWindow {
    regs: [u32; 256],
}

impl MockRegWindow {
    pub fn new() -> Self { Self { regs: [0; 256] } }
    /// Raw register value at a byte offset (for test assertions).
    pub fn get(&self, offset: u32) -> u32 {
        self.regs[(offset / 4) as usize % self.regs.len()]
    }
}

impl Default for MockRegWindow {
    fn default() -> Self { Self::new() }
}

impl RegWindow for MockRegWindow {
    fn read(&mut self, offset: u32) -> u32 { self.get(offset) }
    fn write(&mut self, offset: u32, value: u32) {
        self.regs[(offset / 4) as usize % self.regs.len()] = value;
    }
}

/// Volatile MMIO register window at `base` (physical address, identity
/// mapped). Only use after the APIC page is mapped; on the host this
/// struct is never constructed.
pub struct VolatileRegWindow {
    base: usize,
}

impl VolatileRegWindow {
    /// # Safety
    /// `base` must be the mapped base address of the register page, and
    /// the page must be mapped writable/uncached for the APIC's lifetime.
    pub const unsafe fn new(base: usize) -> Self { Self { base } }
}

impl RegWindow for VolatileRegWindow {
    fn read(&mut self, offset: u32) -> u32 {
        unsafe { core::ptr::read_volatile((self.base + offset as usize) as *const u32) }
    }
    fn write(&mut self, offset: u32, value: u32) {
        unsafe { core::ptr::write_volatile((self.base + offset as usize) as *mut u32, value) }
    }
}

// ---- Local APIC -------------------------------------------------------

/// Default local APIC base address.
pub const LAPIC_BASE: u64 = 0xFEE0_0000;
/// Default IOAPIC base address.
pub const IOAPIC_BASE: u64 = 0xFEC0_0000;

/// Local APIC register offsets (bytes).
pub mod lapic {
    pub const ID: u32 = 0x020;
    pub const VERSION: u32 = 0x030;
    pub const TPR: u32 = 0x080;
    pub const EOI: u32 = 0x0B0;
    pub const SPURIOUS: u32 = 0x0F0;
    pub const ESR: u32 = 0x280;
    pub const ICR_LOW: u32 = 0x300;
    pub const ICR_HIGH: u32 = 0x310;
    pub const LVT_TIMER: u32 = 0x320;
    pub const LVT_THERMAL: u32 = 0x330;
    pub const LVT_PERF: u32 = 0x340;
    pub const LVT_LINT0: u32 = 0x350;
    pub const LVT_LINT1: u32 = 0x360;
    pub const LVT_ERROR: u32 = 0x370;
    pub const TIMER_INITIAL_COUNT: u32 = 0x380;
    pub const TIMER_CURRENT_COUNT: u32 = 0x390;
    pub const TIMER_DIVIDE: u32 = 0x3E0;
}

/// Spurious-interrupt register: software-enable bit.
pub const SPURIOUS_ENABLE: u32 = 0x100;
/// LVT timer modes.
pub const LVT_TIMER_ONE_SHOT: u32 = 0x0000_0000;
pub const LVT_TIMER_PERIODIC: u32 = 0x0002_0000;
pub const LVT_TIMER_TSC_DEADLINE: u32 = 0x0004_0000;
/// LVT mask bit (1 = masked).
pub const LVT_MASKED: u32 = 0x0001_0000;

/// Divide-config encoding for the APIC timer divide register
/// (bits 0, 1 and 3). Returns `None` for unsupported divisors.
pub const fn timer_divide_config(divide: u32) -> Option<u32> {
    match divide {
        1 => Some(0b1011),
        2 => Some(0b0000),
        4 => Some(0b0001),
        8 => Some(0b0010),
        16 => Some(0b0011),
        32 => Some(0b1000),
        64 => Some(0b1001),
        128 => Some(0b1010),
        _ => None,
    }
}

/// Local APIC, generic over its register window.
pub struct LocalApic<R: RegWindow> {
    regs: R,
}

impl<R: RegWindow> LocalApic<R> {
    pub fn new(regs: R) -> Self { Self { regs } }
    pub fn regs(&mut self) -> &mut R { &mut self.regs }

    pub fn id(&mut self) -> u32 { self.regs.read(lapic::ID) >> 24 }
    pub fn version(&mut self) -> u32 { self.regs.read(lapic::VERSION) }

    /// Enable the APIC: set the spurious vector and the enable bit.
    /// `spurious_vector` is the vector for spurious interrupts (39 is
    /// the conventional choice when the PIC is still present).
    pub fn enable(&mut self, spurious_vector: u8) {
        self.regs.write(lapic::SPURIOUS, SPURIOUS_ENABLE | spurious_vector as u32);
    }

    /// Disable the APIC (clear the enable bit, keep the vector).
    pub fn disable(&mut self) {
        let cur = self.regs.read(lapic::SPURIOUS);
        self.regs.write(lapic::SPURIOUS, cur & !SPURIOUS_ENABLE);
    }

    /// Signal end-of-interrupt (any value works; 0 by convention).
    pub fn eoi(&mut self) {
        self.regs.write(lapic::EOI, 0);
    }

    /// Program the APIC timer. `divide` must be 1, 2, 4, 8, 16, 32, 64
    /// or 128. `mode` is one of `LVT_TIMER_*`. The timer fires after
    /// `initial_count` ticks of the (bus-clock / divide) clock.
    pub fn timer(&mut self, vector: u8, mode: u32, divide: u32, initial_count: u32) -> Result<(), u32> {
        let div = timer_divide_config(divide).ok_or(divide)?;
        self.regs.write(lapic::TIMER_DIVIDE, div);
        self.regs.write(lapic::LVT_TIMER, vector as u32 | mode);
        self.regs.write(lapic::TIMER_INITIAL_COUNT, initial_count);
        Ok(())
    }

    /// One-shot timer: fires once after `initial_count` ticks.
    pub fn timer_one_shot(&mut self, vector: u8, divide: u32, initial_count: u32) -> Result<(), u32> {
        self.timer(vector, LVT_TIMER_ONE_SHOT, divide, initial_count)
    }

    /// Periodic timer: fires every `initial_count` ticks.
    pub fn timer_periodic(&mut self, vector: u8, divide: u32, initial_count: u32) -> Result<(), u32> {
        self.timer(vector, LVT_TIMER_PERIODIC, divide, initial_count)
    }

    pub fn timer_current_count(&mut self) -> u32 {
        self.regs.read(lapic::TIMER_CURRENT_COUNT)
    }

    /// Mask the local APIC timer (stops timer interrupts).
    pub fn timer_mask(&mut self) {
        let cur = self.regs.read(lapic::LVT_TIMER);
        self.regs.write(lapic::LVT_TIMER, cur | LVT_MASKED);
    }
}

// ---- IOAPIC -----------------------------------------------------------

/// IOAPIC MMIO offsets (bytes).
pub mod ioapic {
    pub const IOREGSEL: u32 = 0x00;
    pub const IOWIN: u32 = 0x10;
    /// Register indices (selected via IOREGSEL).
    pub const ID: u32 = 0x00;
    pub const VERSION: u32 = 0x01;
    pub const ARBITRATION_ID: u32 = 0x02;
    /// Redirection table entries start here: entry `n` is at
    /// `0x10 + 2*n` (low) and `0x11 + 2*n` (high).
    pub const REDIRECTION_BASE: u32 = 0x10;
}

/// Delivery modes for a redirection entry.
pub const DELIVERY_FIXED: u32 = 0;
pub const DELIVERY_LOWEST_PRIORITY: u32 = 1;
pub const DELIVERY_SMI: u32 = 2;
pub const DELIVERY_NMI: u32 = 4;
pub const DELIVERY_INIT: u32 = 5;
pub const DELIVERY_EXTINT: u32 = 7;

/// Build the low 32 bits of an IOAPIC redirection entry.
/// `polarity`: 0 = high-active, 1 = low-active. `trigger`: 0 = edge,
/// 1 = level. `dest_mode`: 0 = physical, 1 = logical.
pub const fn redirection_entry_low(
    vector: u8,
    delivery_mode: u32,
    dest_mode: u32,
    polarity: u32,
    trigger: u32,
    masked: bool,
) -> u32 {
    vector as u32
        | ((delivery_mode & 0x7) << 8)
        | ((dest_mode & 0x1) << 11)
        | ((polarity & 0x1) << 13)
        | ((trigger & 0x1) << 15)
        | ((masked as u32) << 16)
}

/// High 32 bits of a redirection entry: destination APIC ID.
pub const fn redirection_entry_high(dest_apic_id: u8) -> u32 {
    (dest_apic_id as u32) << 24
}

/// In-memory IOAPIC window for host tests. Models the indirect access:
/// writes to IOREGSEL select a register, reads/writes to IOWIN then
/// hit the selected register. A flat `MockRegWindow` cannot model this
/// (every entry would alias the same IOWIN cell), so IOAPIC tests use
/// this window instead.
pub struct MockIoApicWindow {
    selected: u32,
    regs: [u32; 256],
}

impl MockIoApicWindow {
    pub fn new() -> Self { Self { selected: 0, regs: [0; 256] } }
    /// Raw value of IOAPIC register `index`.
    pub fn get_reg(&self, index: u32) -> u32 {
        self.regs[index as usize % self.regs.len()]
    }
}

impl Default for MockIoApicWindow {
    fn default() -> Self { Self::new() }
}

impl RegWindow for MockIoApicWindow {
    fn read(&mut self, offset: u32) -> u32 {
        match offset {
            ioapic::IOREGSEL => self.selected,
            ioapic::IOWIN => self.get_reg(self.selected),
            _ => self.regs[(offset / 4) as usize % self.regs.len()],
        }
    }
    fn write(&mut self, offset: u32, value: u32) {
        match offset {
            ioapic::IOREGSEL => self.selected = value,
            ioapic::IOWIN => self.regs[self.selected as usize % self.regs.len()] = value,
            _ => self.regs[(offset / 4) as usize % self.regs.len()] = value,
        }
    }
}

/// IOAPIC, generic over its MMIO register window.
pub struct IoApic<R: RegWindow> {
    regs: R,
}

impl<R: RegWindow> IoApic<R> {
    pub fn new(regs: R) -> Self { Self { regs } }

    fn read_reg(&mut self, index: u32) -> u32 {
        self.regs.write(ioapic::IOREGSEL, index);
        self.regs.read(ioapic::IOWIN)
    }

    fn write_reg(&mut self, index: u32, value: u32) {
        self.regs.write(ioapic::IOREGSEL, index);
        self.regs.write(ioapic::IOWIN, value);
    }

    pub fn id(&mut self) -> u32 { self.read_reg(ioapic::ID) >> 24 }
    /// Maximum redirection entry index (from the version register).
    pub fn max_redirection_entry(&mut self) -> u32 {
        (self.read_reg(ioapic::VERSION) >> 16) & 0xFF
    }

    /// Read redirection entry `n` as (low, high).
    pub fn redirection_entry(&mut self, n: u32) -> (u32, u32) {
        let base = ioapic::REDIRECTION_BASE + 2 * n;
        (self.read_reg(base), self.read_reg(base + 1))
    }

    /// Write redirection entry `n` from (low, high).
    pub fn set_redirection_entry(&mut self, n: u32, low: u32, high: u32) {
        let base = ioapic::REDIRECTION_BASE + 2 * n;
        self.write_reg(base, low);
        self.write_reg(base + 1, high);
    }

    /// Route IRQ `n` to `vector` on APIC `dest_apic_id` (fixed delivery,
    /// physical destination, high-active edge trigger, unmasked).
    pub fn route_irq(&mut self, n: u32, vector: u8, dest_apic_id: u8) {
        let low = redirection_entry_low(vector, DELIVERY_FIXED, 0, 0, 0, false);
        self.set_redirection_entry(n, low, redirection_entry_high(dest_apic_id));
    }

    /// Mask redirection entry `n` (stops that IRQ reaching any CPU).
    pub fn mask_irq(&mut self, n: u32) {
        let (low, high) = self.redirection_entry(n);
        self.set_redirection_entry(n, low | (1 << 16), high);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lapic_register_offsets() {
        assert_eq!(lapic::ID, 0x020);
        assert_eq!(lapic::EOI, 0x0B0);
        assert_eq!(lapic::SPURIOUS, 0x0F0);
        assert_eq!(lapic::LVT_TIMER, 0x320);
        assert_eq!(lapic::TIMER_INITIAL_COUNT, 0x380);
        assert_eq!(lapic::TIMER_CURRENT_COUNT, 0x390);
        assert_eq!(lapic::TIMER_DIVIDE, 0x3E0);
        assert_eq!(LAPIC_BASE, 0xFEE0_0000);
        assert_eq!(IOAPIC_BASE, 0xFEC0_0000);
    }

    #[test]
    fn apic_enable_and_eoi() {
        let mut apic = LocalApic::new(MockRegWindow::new());
        apic.enable(0xFF);
        assert_eq!(apic.regs.get(lapic::SPURIOUS), SPURIOUS_ENABLE | 0xFF);
        apic.eoi();
        assert_eq!(apic.regs.get(lapic::EOI), 0);
        apic.disable();
        assert_eq!(apic.regs.get(lapic::SPURIOUS) & SPURIOUS_ENABLE, 0);
    }

    #[test]
    fn apic_timer_periodic() {
        let mut apic = LocalApic::new(MockRegWindow::new());
        apic.timer_periodic(0x30, 16, 1_000_000).unwrap();
        assert_eq!(apic.regs.get(lapic::TIMER_DIVIDE), 0b0011);
        assert_eq!(apic.regs.get(lapic::LVT_TIMER), 0x30 | LVT_TIMER_PERIODIC);
        assert_eq!(apic.regs.get(lapic::TIMER_INITIAL_COUNT), 1_000_000);
    }

    #[test]
    fn apic_timer_one_shot_and_bad_divide() {
        let mut apic = LocalApic::new(MockRegWindow::new());
        apic.timer_one_shot(0x31, 1, 500).unwrap();
        assert_eq!(apic.regs.get(lapic::TIMER_DIVIDE), 0b1011);
        assert_eq!(apic.regs.get(lapic::LVT_TIMER), 0x31 | LVT_TIMER_ONE_SHOT);
        assert_eq!(apic.timer_one_shot(0x31, 3, 500), Err(3));
        assert_eq!(apic.timer_one_shot(0x31, 0, 500), Err(0));
    }

    #[test]
    fn divide_config_table() {
        assert_eq!(timer_divide_config(1), Some(0b1011));
        assert_eq!(timer_divide_config(2), Some(0b0000));
        assert_eq!(timer_divide_config(4), Some(0b0001));
        assert_eq!(timer_divide_config(8), Some(0b0010));
        assert_eq!(timer_divide_config(16), Some(0b0011));
        assert_eq!(timer_divide_config(32), Some(0b1000));
        assert_eq!(timer_divide_config(64), Some(0b1001));
        assert_eq!(timer_divide_config(128), Some(0b1010));
        assert_eq!(timer_divide_config(3), None);
        assert_eq!(timer_divide_config(256), None);
    }

    #[test]
    fn ioapic_redirection_entry_builder() {
        let low = redirection_entry_low(0x21, DELIVERY_FIXED, 0, 0, 0, false);
        assert_eq!(low, 0x21);
        let masked = redirection_entry_low(0x21, DELIVERY_FIXED, 0, 0, 0, true);
        assert_eq!(masked, 0x21 | (1 << 16));
        let level_low = redirection_entry_low(0x21, DELIVERY_FIXED, 0, 1, 1, false);
        assert_eq!(level_low, 0x21 | (1 << 13) | (1 << 15));
        assert_eq!(redirection_entry_high(0x02), 0x0200_0000);
    }

    #[test]
    fn ioapic_route_and_mask_roundtrip() {
        let mut io = IoApic::new(MockIoApicWindow::new());
        io.route_irq(1, 0x21, 0x00);
        let (low, high) = io.redirection_entry(1);
        assert_eq!(low, 0x21);
        assert_eq!(high, 0);
        io.mask_irq(1);
        let (low, _) = io.redirection_entry(1);
        assert_eq!(low & (1 << 16), 1 << 16);
    }

    #[test]
    fn ioapic_entries_do_not_alias() {
        let mut io = IoApic::new(MockIoApicWindow::new());
        io.route_irq(0, 0x20, 0x00);
        io.route_irq(1, 0x21, 0x00);
        io.route_irq(2, 0x22, 0x00);
        assert_eq!(io.redirection_entry(0).0, 0x20);
        assert_eq!(io.redirection_entry(1).0, 0x21);
        assert_eq!(io.redirection_entry(2).0, 0x22);
    }
}
