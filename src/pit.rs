//! PIT (8253/8254) programmable interval timer, channel 0.
//!
//! Same divisor math as `rustos-drivers::pit` (kept identical on
//! purpose — the two crates must never disagree about the timer):
//! input clock 1_193_182 Hz, channel 0 port 0x40, command port 0x43,
//! mode 3 (square wave) is command byte 0x36.
//!
//! This crate adds the interrupt-side half the drivers crate does not
//! have: [`PitTimer`], which counts ticks and tells the caller when a
//! scheduler quantum has elapsed, so the timer IRQ handler (vector 32)
//! can call `rustos-scheduler`'s `tick()` once per quantum instead of
//! once per PIT tick.

use crate::ports::PortIo;

pub const PIT_FREQUENCY_HZ: u32 = 1_193_182;
pub const PIT_CHANNEL0_PORT: u16 = 0x40;
pub const PIT_COMMAND_PORT: u16 = 0x43;
pub const PIT_MODE_SQUARE_WAVE: u8 = 0x36;

/// Divisor for a target interrupt frequency. Clamped to 1..=65535
/// (a divisor of 0 in hardware means 65536).
pub const fn divisor_for_hz(hz: u32) -> u16 {
    if hz == 0 { return 0xFFFF; }
    let d = PIT_FREQUENCY_HZ / hz;
    if d == 0 { 1 } else if d > 0xFFFF { 0xFFFF } else { d as u16 }
}

/// Actual frequency produced by a divisor (0 means 65536).
pub const fn frequency_for_divisor(divisor: u16) -> u32 {
    let d = if divisor == 0 { 65536u32 } else { divisor as u32 };
    PIT_FREQUENCY_HZ / d
}

/// Program channel 0 to `hz`. Writes 0x36 then divisor low/high.
pub fn init<P: PortIo>(ports: &mut P, hz: u32) {
    let divisor = divisor_for_hz(hz);
    ports.write_u8(PIT_COMMAND_PORT, PIT_MODE_SQUARE_WAVE);
    ports.write_u8(PIT_CHANNEL0_PORT, (divisor & 0xFF) as u8);
    ports.write_u8(PIT_CHANNEL0_PORT, (divisor >> 8) as u8);
}

/// PIT ticks in `ms` milliseconds at `hz`.
pub const fn ms_to_ticks(ms: u32, hz: u32) -> u32 { ms * hz / 1000 }
/// Milliseconds in `ticks` ticks at `hz`.
pub const fn ticks_to_ms(ticks: u32, hz: u32) -> u32 {
    if hz == 0 { 0 } else { ticks * 1000 / hz }
}

/// What a PIT tick means for the scheduler.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TickOutcome {
    /// A plain tick; no quantum boundary crossed.
    Tick,
    /// A scheduler quantum elapsed — the scheduler should run `tick()`
    /// (e.g. pre-empt the current task) now.
    QuantumElapsed,
}

/// Counts PIT ticks and groups them into scheduler quanta.
///
/// The timer IRQ handler calls [`PitTimer::on_tick`] once per interrupt;
/// when it returns [`TickOutcome::QuantumElapsed`] the handler calls the
/// scheduler's `tick()`. `quantum_ticks = 10` at 100 Hz is a 100 ms
/// quantum; `quantum_ticks = 1` pre-empts on every tick.
#[derive(Debug, Clone, Copy)]
pub struct PitTimer {
    ticks: u64,
    ticks_in_quantum: u64,
    quantum_ticks: u64,
}

impl PitTimer {
    /// `quantum_ticks` PIT ticks per scheduler quantum (0 behaves as 1).
    pub fn new(quantum_ticks: u64) -> Self {
        Self { ticks: 0, ticks_in_quantum: 0, quantum_ticks: quantum_ticks.max(1) }
    }

    pub fn ticks(&self) -> u64 { self.ticks }
    pub fn quantum_ticks(&self) -> u64 { self.quantum_ticks }
    /// Whole quanta elapsed so far.
    pub fn quanta_elapsed(&self) -> u64 { self.ticks / self.quantum_ticks }

    /// Record one timer interrupt. Returns whether a quantum elapsed.
    pub fn on_tick(&mut self) -> TickOutcome {
        self.ticks += 1;
        self.ticks_in_quantum += 1;
        if self.ticks_in_quantum >= self.quantum_ticks {
            self.ticks_in_quantum = 0;
            TickOutcome::QuantumElapsed
        } else {
            TickOutcome::Tick
        }
    }
}

impl Default for PitTimer {
    fn default() -> Self { Self::new(10) }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ports::MockPorts;

    #[test]
    fn divisor_and_frequency() {
        assert_eq!(divisor_for_hz(100), 11931);
        assert_eq!(divisor_for_hz(1000), 1193);
        assert_eq!(divisor_for_hz(1), 0xFFFF); // clamped: 1193182 > 65535
        assert_eq!(divisor_for_hz(0), 0xFFFF);
        assert_eq!(divisor_for_hz(2_000_000), 1);
        let f = frequency_for_divisor(divisor_for_hz(100));
        assert!((99..=101).contains(&f), "freq for 100 Hz divisor: {f}");
        assert_eq!(frequency_for_divisor(0), 18); // 1193182 / 65536
    }

    #[test]
    fn init_writes_command_and_divisor() {
        let mut ports = MockPorts::new();
        init(&mut ports, 100);
        assert!(ports.wrote(PIT_COMMAND_PORT, PIT_MODE_SQUARE_WAVE));
        assert_eq!(ports.get(PIT_CHANNEL0_PORT), (11931u16 >> 8) as u8); // high byte last
        assert!(ports.wrote(PIT_CHANNEL0_PORT, (11931u16 & 0xFF) as u8));
    }

    #[test]
    fn tick_conversions() {
        assert_eq!(ms_to_ticks(1000, 100), 100);
        assert_eq!(ticks_to_ms(100, 100), 1000);
        assert_eq!(ticks_to_ms(50, 0), 0);
    }

    #[test]
    fn quantum_boundaries() {
        let mut t = PitTimer::new(10);
        for _ in 0..9 {
            assert_eq!(t.on_tick(), TickOutcome::Tick);
        }
        assert_eq!(t.on_tick(), TickOutcome::QuantumElapsed);
        assert_eq!(t.ticks(), 10);
        assert_eq!(t.quanta_elapsed(), 1);
        for _ in 0..9 {
            assert_eq!(t.on_tick(), TickOutcome::Tick);
        }
        assert_eq!(t.on_tick(), TickOutcome::QuantumElapsed);
        assert_eq!(t.quanta_elapsed(), 2);
    }

    #[test]
    fn quantum_of_one_preempts_every_tick() {
        let mut t = PitTimer::new(1);
        assert_eq!(t.on_tick(), TickOutcome::QuantumElapsed);
        assert_eq!(t.on_tick(), TickOutcome::QuantumElapsed);
        assert_eq!(t.quanta_elapsed(), 2);
    }

    #[test]
    fn quantum_zero_behaves_as_one() {
        let mut t = PitTimer::new(0);
        assert_eq!(t.quantum_ticks(), 1);
        assert_eq!(t.on_tick(), TickOutcome::QuantumElapsed);
    }
}
