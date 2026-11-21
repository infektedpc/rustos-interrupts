//! rustos-interrupts — no_std interrupt plumbing for a Rust OS.
//!
//! * `ports` — `PortIo` trait, `MockPorts`, `RealPorts` (x86_64).
//! * `pic` — 8259 PIC pair: remap to vectors 32–47, mask/unmask,
//!   EOI, spurious-IRQ detection.
//! * `pit` — 8253/8254 timer: divisor math (identical to
//!   `rustos-drivers::pit`) plus `PitTimer`, which groups ticks into
//!   scheduler quanta.
//! * `apic` — local APIC and IOAPIC register layout over a `RegWindow`
//!   (mocked in tests, volatile MMIO on hardware), plus `init_apic`
//!   and the PIC <-> APIC mode switch (`switch_to_apic` /
//!   `switch_to_pic`) — exactly one controller owns IRQ0/IRQ1.
//! * `timer` — the IRQ0 handler state: [`TimerIrq`] advances the
//!   [`PitTimer`] and runs the scheduler tick only at quantum
//!   boundaries.
//! * `keyboard` — the IRQ1 handler: PS/2 set-1 decode into a
//!   64-entry [`KeyRing`] the kernel drains.
//! * `idt` — 16-byte IDT entry layout, vector map, handler registry.
//! * `handler` — dispatch: vector -> handler -> EOI.
//!
//! The crate is `#![no_std]` outside tests so it can link into a
//! kernel. Tests run on the host with std. No external crates.

#![cfg_attr(not(test), no_std)]

pub mod apic;
pub mod handler;
pub mod idt;
pub mod keyboard;
pub mod pic;
pub mod pit;
pub mod ports;
pub mod timer;

pub use apic::{
    init_apic, switch_to_apic, switch_to_pic, ControllerMode, IoApic, LocalApic,
    MockIoApicWindow, MockRegWindow, RegWindow, VolatileRegWindow, APIC_SPURIOUS_VECTOR,
};
pub use handler::{dispatch, ChainedEoi, DispatchOutcome, Eoi, RecordingEoi};
pub use idt::{HandlerTable, Idt, IdtEntry};
pub use keyboard::{KeyEvent, KeyRing, KeyboardDecoder, KeyboardIrq, KEY_RING_CAPACITY};
pub use pic::Pic;
pub use pit::PitTimer;
pub use ports::{MockPorts, PortIo};
pub use timer::{TimerIrq, TimerOutcome};
#[cfg(target_arch = "x86_64")]
pub use ports::RealPorts;
