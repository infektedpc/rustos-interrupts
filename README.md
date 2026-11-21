# rustos-interrupts

`no_std` interrupt plumbing for a Rust OS. Core only, no external
crates. Host-testable: every controller is generic over a port/MMIO
abstraction, and tests use in-memory mocks.

| Module | Hardware / job | Notes |
|---|---|---|
| `pic` | 8259 PIC pair, 0x20/0x21 + 0xA0/0xA1 | remap to vectors 32–47, mask/unmask, EOI, spurious-IRQ detection |
| `pit` | PIT 8253/8254, channel 0, 1_193_182 Hz | divisor math (identical to `rustos-drivers::pit`), `PitTimer` quantum grouping |
| `apic` | Local APIC 0xFEE00000, IOAPIC 0xFEC00000 | register offsets, timer (one-shot/periodic), EOI, redirection entries |
| `idt` | IDT entry layout (16 bytes), vector map | exception vectors 0–31, PIC 32–47, APIC 48+, handler registry |
| `handler` | Dispatch: vector → handler → EOI | routes EOI to PIC or APIC, spurious-vector handling |
| `ports` | `PortIo` trait, `MockPorts`, `RealPorts` (x86_64) | same contract as `rustos-drivers::ports`, self-contained |

## IRQ map

| Vector | Source | Notes |
|-------:|--------|-------|
| 0–31 | CPU exceptions | divide error, breakpoint (3), double fault (8), page fault (14), … |
| 32 | PIC IRQ0 — PIT timer | drives the scheduler tick (see below) |
| 33 | PIC IRQ1 — PS/2 keyboard | |
| 34–38 | PIC IRQ2–6 | cascade, COM2, COM1, LPT2, floppy |
| 39 | PIC IRQ7 — spurious | no EOI when spurious |
| 40–46 | PIC IRQ8–14 | RTC, …, IDE |
| 47 | PIC IRQ15 — spurious | master-only EOI when spurious |
| 48+ | Local APIC | APIC timer, LINT0/1, error, IPIs, MSI |

## How the timer IRQ drives the scheduler tick

The PIT fires IRQ0 (vector 32) at the programmed rate — 100 Hz is
the usual choice (`pit::divisor_for_hz(100)` = 11931). The kernel's
vector-32 stub calls `handler::dispatch(32, table, eoi)`, and the
registered handler does:

```rust
// in the kernel's timer handler (pseudo-code):
fn timer_handler(_vector: u8) {
    // PIT_TIMER is a `PitTimer` with, say, 10 ticks per quantum.
    if PIT_TIMER.on_tick() == TickOutcome::QuantumElapsed {
        scheduler::tick(); // pre-empt the current task (rustos-scheduler)
    }
}
```

So the scheduler runs once per quantum (100 ms at 100 Hz / 10 ticks),
not once per PIT tick. `PitTimer` keeps the count; the scheduler
itself lives in `rustos-scheduler` — this crate only owns the
interrupt side and never depends on it.

## Wiring it into the kernel

```rust
use rustos_interrupts::{Pic, RealPorts, HandlerTable, dispatch};
use rustos_interrupts::idt::TIMER_VECTOR;
use rustos_interrupts::pit;

// 1. Remap the PICs before enabling any hardware interrupt.
let mut pic = Pic::new(RealPorts);
pic.remap_default();          // master -> 32, slave -> 40
pic.unmask_irq(0);            // timer
pic.unmask_irq(1);            // keyboard

// 2. Program the PIT for 100 Hz.
let mut ports = RealPorts;
rustos_interrupts::pit::init(&mut ports, 100);

// 3. Register the timer handler; the IDT stub for vector 32 calls
//    dispatch(32, &table, &mut pic) and the PIC gets its EOI.
let mut table = HandlerTable::new();
table.register(TIMER_VECTOR, timer_handler);

// 4. Enable the local APIC instead of (or as well as) the PIC by
//    writing the spurious vector + enable bit — see `apic::LocalApic`.
```

The existing `rustos-kernel` IDT only serves CPU exceptions until
this step is done: its `interrupts.rs` notes the PICs were not
remapped yet, which is exactly what `Pic::remap_default` fixes.

## Build / test

```sh
export PATH="$HOME/.cargo/bin:$PATH"
cargo test
```

## Limitations

- **Not live-boot-tested.** QEMU is not installed on this VM, so the
  PIC remap sequence, APIC timer and dispatch have only been verified
  against `MockPorts` / `MockRegWindow` — the register writes are
  asserted byte-for-byte, but no real hardware has run them yet.
- No MSI/MSI-X support.
- No IOAPIC redirection-table programming on real hardware yet: the
  entry builder (`redirection_entry_low/high`, `IoApic::route_irq`)
  is host-tested, but no kernel wires it to a live IOAPIC, and the
  bases are not discovered from ACPI/MADT (they default to
  0xFEE00000 / 0xFEC00000).
- No APIC-base MSR read; `LocalApic` trusts the caller to hand it a
  mapped `VolatileRegWindow`.
- The PIT divisor math is deliberately duplicated from
  `rustos-drivers::pit` (both crates are standalone, no sibling
  dependency) — if the math ever changes, change it in both places;
  the tests in both crates assert the same values (100 Hz → 11931).
- `PitTimer` counts ticks; it does not calibrate the APIC timer
  against the PIT, which is how a real kernel derives the APIC bus
  frequency.
