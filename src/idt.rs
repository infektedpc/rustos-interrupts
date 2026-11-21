//! Interrupt Descriptor Table (IDT) entry layout and vector map.
//!
//! This is the *layout* half of interrupt handling: what an IDT entry
//! looks like in memory and which vector means what. The *handlers*
//! themselves live in `rustos-kernel` (which owns the real IDT via the
//! `x86_64` crate and installs exception handlers for breakpoint,
//! double fault and page fault) and in [`crate::handler`].
//!
//! Vector map:
//!
//! | Vectors | Source |
//! |--------:|--------|
//! | 0–31 | CPU exceptions (see constants below) |
//! | 32–47 | 8259 PIC IRQs, after remap (master 32–39, slave 40–47) |
//! | 48+ | Local APIC (timer, LINT, error, IPIs, MSI) |
//!
//! An IDT entry is 16 bytes: offset low (16), segment selector (16),
//! IST (3 bits) + reserved (5), type/attributes (8), offset mid (16),
//! offset high (32), reserved (32).

/// CPU exception vectors.
pub const DIVIDE_ERROR: u8 = 0;
pub const DEBUG: u8 = 1;
pub const NON_MASKABLE_INTERRUPT: u8 = 2;
pub const BREAKPOINT: u8 = 3;
pub const OVERFLOW: u8 = 4;
pub const BOUND_RANGE_EXCEEDED: u8 = 5;
pub const INVALID_OPCODE: u8 = 6;
pub const DEVICE_NOT_AVAILABLE: u8 = 7;
pub const DOUBLE_FAULT: u8 = 8;
pub const COPROCESSOR_SEGMENT_OVERRUN: u8 = 9;
pub const INVALID_TSS: u8 = 10;
pub const SEGMENT_NOT_PRESENT: u8 = 11;
pub const STACK_SEGMENT_FAULT: u8 = 12;
pub const GENERAL_PROTECTION_FAULT: u8 = 13;
pub const PAGE_FAULT: u8 = 14;
pub const X87_FLOATING_POINT: u8 = 16;
pub const ALIGNMENT_CHECK: u8 = 17;
pub const MACHINE_CHECK: u8 = 18;
pub const SIMD_FLOATING_POINT: u8 = 19;
pub const VIRTUALIZATION: u8 = 20;
pub const CONTROL_PROTECTION: u8 = 21;

/// First vector used by the remapped PICs.
pub const PIC_IRQ_BASE: u8 = 32;
/// First vector reserved for APIC sources.
pub const APIC_VECTOR_BASE: u8 = 48;

/// Timer IRQ (PIC IRQ0) after remap — drives the scheduler tick.
pub const TIMER_VECTOR: u8 = 32;
/// Keyboard IRQ (PIC IRQ1) after remap.
pub const KEYBOARD_VECTOR: u8 = 33;
/// Spurious IRQ vectors (PIC IRQ7 / IRQ15 after remap).
pub const SPURIOUS_MASTER_VECTOR: u8 = 39;
pub const SPURIOUS_SLAVE_VECTOR: u8 = 47;

/// Number of IDT entries.
pub const IDT_ENTRIES: usize = 256;

/// Gate type/attribute byte: present, DPL 0, interrupt gate (0xE).
pub const INTERRUPT_GATE: u8 = 0x8E;
/// Gate type/attribute byte: present, DPL 0, trap gate (0xF).
pub const TRAP_GATE: u8 = 0x8F;
/// Gate type/attribute byte: present, DPL 3, interrupt gate — for
/// user-callable gates (e.g. `int3` breakpoint).
pub const INTERRUPT_GATE_USER: u8 = 0xEE;

/// One 16-byte IDT entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(C)]
pub struct IdtEntry {
    offset_low: u16,
    selector: u16,
    ist: u8,
    type_attr: u8,
    offset_mid: u16,
    offset_high: u32,
    reserved: u32,
}

impl IdtEntry {
    /// A zero (absent) entry.
    pub const fn empty() -> Self {
        Self { offset_low: 0, selector: 0, ist: 0, type_attr: 0, offset_mid: 0, offset_high: 0, reserved: 0 }
    }

    /// Entry for `handler_addr` in code segment `selector`, with the
    /// given gate type and IST index (0 = no IST stack switch).
    pub const fn new(handler_addr: u64, selector: u16, ist: u8, type_attr: u8) -> Self {
        Self {
            offset_low: handler_addr as u16,
            selector,
            ist: ist & 0x07,
            type_attr,
            offset_mid: (handler_addr >> 16) as u16,
            offset_high: (handler_addr >> 32) as u32,
            reserved: 0,
        }
    }

    /// Interrupt gate (IF cleared on entry) for `handler_addr`.
    pub const fn interrupt_gate(handler_addr: u64, selector: u16) -> Self {
        Self::new(handler_addr, selector, 0, INTERRUPT_GATE)
    }

    /// Trap gate (IF left alone) for `handler_addr`.
    pub const fn trap_gate(handler_addr: u64, selector: u16) -> Self {
        Self::new(handler_addr, selector, 0, TRAP_GATE)
    }

    /// Interrupt gate using IST stack `ist` (1–7; 0 disables it).
    pub const fn interrupt_gate_ist(handler_addr: u64, selector: u16, ist: u8) -> Self {
        Self::new(handler_addr, selector, ist, INTERRUPT_GATE)
    }

    /// Reconstruct the handler address from the three offset fields.
    pub const fn handler_addr(&self) -> u64 {
        (self.offset_low as u64)
            | ((self.offset_mid as u64) << 16)
            | ((self.offset_high as u64) << 32)
    }

    pub const fn selector(&self) -> u16 { self.selector }
    pub const fn ist(&self) -> u8 { self.ist }
    pub const fn type_attr(&self) -> u8 { self.type_attr }
    /// Present bit (bit 7 of the type/attributes byte).
    pub const fn is_present(&self) -> bool { self.type_attr & 0x80 != 0 }
}

/// The full 256-entry table. `load()` is x86_64-only; host tests never
/// call it (executing `lidt` in userspace would fault).
#[derive(Debug, Clone)]
pub struct Idt {
    entries: [IdtEntry; IDT_ENTRIES],
}

impl Idt {
    pub fn new() -> Self { Self { entries: [IdtEntry::empty(); IDT_ENTRIES] } }

    pub fn entry(&self, vector: u8) -> IdtEntry { self.entries[vector as usize] }

    pub fn set_entry(&mut self, vector: u8, entry: IdtEntry) {
        self.entries[vector as usize] = entry;
    }

    /// Install an interrupt gate for `handler_addr` at `vector`.
    pub fn set_interrupt_gate(&mut self, vector: u8, handler_addr: u64, selector: u16) {
        self.set_entry(vector, IdtEntry::interrupt_gate(handler_addr, selector));
    }

    /// Install a trap gate for `handler_addr` at `vector`.
    pub fn set_trap_gate(&mut self, vector: u8, handler_addr: u64, selector: u16) {
        self.set_entry(vector, IdtEntry::trap_gate(handler_addr, selector));
    }

    /// Install an interrupt gate using IST stack `ist` (e.g. the
    /// double-fault handler's dedicated stack from `rustos-kernel`'s GDT).
    pub fn set_interrupt_gate_ist(&mut self, vector: u8, handler_addr: u64, selector: u16, ist: u8) {
        self.set_entry(vector, IdtEntry::interrupt_gate_ist(handler_addr, selector, ist));
    }

    /// Load this table into the CPU. x86_64 only.
    ///
    /// # Safety
    /// The entries must point at valid interrupt handlers; a bad entry
    /// triple-faults the machine. Call once, after the GDT is loaded.
    #[cfg(target_arch = "x86_64")]
    pub unsafe fn load(&self) {
        #[repr(C, packed)]
        struct Descriptor {
            limit: u16,
            base: u64,
        }
        let desc = Descriptor {
            limit: (core::mem::size_of::<Idt>() - 1) as u16,
            base: self as *const Self as u64,
        };
        unsafe {
            core::arch::asm!("lidt [{}]", in(reg) &desc, options(nostack, preserves_flags));
        }
    }
}

impl Default for Idt {
    fn default() -> Self { Self::new() }
}

/// A registered interrupt handler: a plain function taking the vector
/// number it was called for.
pub type HandlerFn = fn(u8);

/// Host-testable handler registry: vector -> handler, 256 slots.
/// The real IDT (above) holds handler *addresses* for the CPU; this
/// table holds Rust functions for the dispatcher in `crate::handler`.
/// A kernel wires the two together: the CPU-facing stub for a vector
/// calls `dispatch(vector, ...)`, which looks the handler up here.
pub struct HandlerTable {
    handlers: [Option<HandlerFn>; IDT_ENTRIES],
}

impl HandlerTable {
    pub fn new() -> Self { Self { handlers: [None; IDT_ENTRIES] } }

    /// Register `handler` for `vector`, returning any previous handler.
    pub fn register(&mut self, vector: u8, handler: HandlerFn) -> Option<HandlerFn> {
        let prev = self.handlers[vector as usize];
        self.handlers[vector as usize] = Some(handler);
        prev
    }

    /// Remove the handler for `vector`, returning it if there was one.
    pub fn unregister(&mut self, vector: u8) -> Option<HandlerFn> {
        let prev = self.handlers[vector as usize];
        self.handlers[vector as usize] = None;
        prev
    }

    pub fn get(&self, vector: u8) -> Option<HandlerFn> {
        self.handlers[vector as usize]
    }

    pub fn is_registered(&self, vector: u8) -> bool {
        self.handlers[vector as usize].is_some()
    }
}

impl Default for HandlerTable {
    fn default() -> Self { Self::new() }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn entry_layout_roundtrip() {
        assert_eq!(core::mem::size_of::<IdtEntry>(), 16);
        let addr = 0xFFFF_8000_0010_1234u64;
        let e = IdtEntry::interrupt_gate(addr, 0x08);
        assert_eq!(e.handler_addr(), addr);
        assert_eq!(e.selector(), 0x08);
        assert_eq!(e.type_attr(), INTERRUPT_GATE);
        assert!(e.is_present());
        assert_eq!(e.ist(), 0);
        assert!(!IdtEntry::empty().is_present());
        assert_eq!(IdtEntry::empty().handler_addr(), 0);
    }

    #[test]
    fn trap_gate_and_ist() {
        let e = IdtEntry::trap_gate(0x1234, 0x08);
        assert_eq!(e.type_attr(), TRAP_GATE);
        assert!(e.is_present());
        let e = IdtEntry::interrupt_gate_ist(0x1234, 0x08, 3);
        assert_eq!(e.ist(), 3);
        // IST is 3 bits: values above 7 are masked.
        let e = IdtEntry::new(0x1234, 0x08, 9, INTERRUPT_GATE);
        assert_eq!(e.ist(), 1);
    }

    #[test]
    fn vector_map_constants() {
        assert_eq!(PIC_IRQ_BASE, 32);
        assert_eq!(APIC_VECTOR_BASE, 48);
        assert_eq!(TIMER_VECTOR, 32);
        assert_eq!(KEYBOARD_VECTOR, 33);
        assert_eq!(SPURIOUS_MASTER_VECTOR, 39);
        assert_eq!(SPURIOUS_SLAVE_VECTOR, 47);
        assert_eq!(DOUBLE_FAULT, 8);
        assert_eq!(PAGE_FAULT, 14);
        assert_eq!(BREAKPOINT, 3);
    }

    #[test]
    fn idt_install_and_read_back() {
        let mut idt = Idt::new();
        assert!(!idt.entry(TIMER_VECTOR).is_present());
        idt.set_interrupt_gate(TIMER_VECTOR, 0x1000, 0x08);
        assert_eq!(idt.entry(TIMER_VECTOR).handler_addr(), 0x1000);
        assert!(idt.entry(TIMER_VECTOR).is_present());
        idt.set_trap_gate(BREAKPOINT, 0x2000, 0x08);
        assert_eq!(idt.entry(BREAKPOINT).type_attr(), TRAP_GATE);
        idt.set_interrupt_gate_ist(DOUBLE_FAULT, 0x3000, 0x08, 1);
        assert_eq!(idt.entry(DOUBLE_FAULT).ist(), 1);
        assert_eq!(core::mem::size_of::<Idt>(), 256 * 16);
    }

    #[test]
    fn handler_table_register_unregister() {
        fn a(_v: u8) {}
        fn b(_v: u8) {}
        // Compare handler identity via address-as-usize: fn-pointer
        // equality itself warns (addresses are not guaranteed unique).
        let addr = |f: Option<HandlerFn>| f.map(|h| h as usize);
        let mut t = HandlerTable::new();
        assert!(!t.is_registered(32));
        assert_eq!(addr(t.register(32, a)), None);
        assert!(t.is_registered(32));
        assert_eq!(addr(t.register(32, b)), Some(a as HandlerFn as usize));
        assert_eq!(addr(t.get(32)), Some(b as HandlerFn as usize));
        assert_eq!(addr(t.unregister(32)), Some(b as HandlerFn as usize));
        assert!(!t.is_registered(32));
        assert_eq!(addr(t.unregister(32)), None);
    }
}
