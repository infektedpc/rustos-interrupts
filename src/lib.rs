//! rustos-interrupts — no_std interrupt plumbing for a Rust OS.
//!
//! * `ports` — `PortIo` trait, `MockPorts`, `RealPorts` (x86_64).
//! * `pic` — 8259 PIC pair: remap to vectors 32–47, mask/unmask,
//!   EOI, spurious-IRQ detection.
//! * `pit` — 8253/8254 timer: divisor math (identical to
//!   `rustos-drivers::pit`) plus `PitTimer`, which groups ticks into
//!   scheduler quanta.
//! * `apic` — local APIC and IOAPIC register layout over a `RegWindow`
//!   (mocked in tests, volatile MMIO on hardware).
//! * `idt` — 16-byte IDT entry layout, vector map, handler registry.
//! * `handler` — dispatch: vector -> handler -> EOI.
//!
//! The crate is `#![no_std]` outside tests so it can link into a
//! kernel. Tests run on the host with std. No external crates.

#![cfg_attr(not(test), no_std)]

pub mod apic;
pub mod handler;
pub mod idt;
pub mod pic;
pub mod pit;
pub mod ports;

pub use apic::{IoApic, LocalApic, MockIoApicWindow, MockRegWindow, RegWindow, VolatileRegWindow};
pub use handler::{dispatch, ChainedEoi, DispatchOutcome, Eoi, RecordingEoi};
pub use idt::{HandlerTable, Idt, IdtEntry};
pub use pic::Pic;
pub use pit::PitTimer;
pub use ports::{MockPorts, PortIo};
#[cfg(target_arch = "x86_64")]
pub use ports::RealPorts;
