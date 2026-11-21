//! Timer IRQ handler state: PIT ticks -> scheduler quanta.
//!
//! The PIT itself only does frequency math (see [`crate::pit`]); this
//! module is the part that runs on a real IRQ0 (vector 32). The kernel's
//! vector-32 stub calls [`TimerIrq::handle_irq`] once per interrupt.
//! `handle_irq` advances the [`PitTimer`] and reports whether a
//! scheduler quantum elapsed; [`TimerIrq::handle_irq_with`] additionally
//! runs a caller-supplied callback (the scheduler tick) **only** at a
//! quantum boundary, never before.
//!
//! Quantum size: the scheduler's timeslice is 10 ms
//! (`rustos-scheduler::timer::TIMESLICE_MS`), so at 100 Hz one PIT tick
//! *is* one quantum (`ticks_per_timeslice(100) == 1`). The kernel
//! therefore builds this with `quantum_ticks = 1` at 100 Hz and
//! `Scheduler::tick()` runs on every tick. At 1000 Hz the same code
//! uses `quantum_ticks = 10` and the tick runs every tenth interrupt.
//! The default [`PitTimer`] quantum (10) is the 1000 Hz case, not a
//! second layer of quanta on top of the scheduler's own.

use crate::pit::{PitTimer, TickOutcome};

/// What one timer IRQ did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimerOutcome {
    /// A plain PIT tick; no quantum boundary crossed.
    Tick,
    /// A scheduler quantum elapsed — the scheduler tick is due.
    QuantumElapsed,
}

/// State for the IRQ0 handler. Owns the [`PitTimer`] that groups PIT
/// ticks into scheduler quanta and counts the quanta elapsed.
#[derive(Debug)]
pub struct TimerIrq {
    timer: PitTimer,
    quanta: u64,
}

impl TimerIrq {
    /// `quantum_ticks` PIT ticks per scheduler quantum (0 behaves as 1).
    /// Pass `rustos-scheduler`'s `ticks_per_timeslice(hz)` so the
    /// quantum here and the scheduler's own quantum agree.
    pub fn new(quantum_ticks: u64) -> Self {
        Self { timer: PitTimer::new(quantum_ticks), quanta: 0 }
    }

    /// PIT ticks per scheduler quantum.
    pub fn quantum_ticks(&self) -> u64 {
        self.timer.quantum_ticks()
    }

    /// Total PIT ticks handled.
    pub fn ticks(&self) -> u64 {
        self.timer.ticks()
    }

    /// Whole scheduler quanta elapsed so far.
    pub fn quanta(&self) -> u64 {
        self.quanta
    }

    /// Handle one IRQ0: advance the tick counter and report whether a
    /// scheduler quantum elapsed. Pure logic — no port I/O, no EOI;
    /// the caller (kernel stub) sends EOI after this returns.
    pub fn handle_irq(&mut self) -> TimerOutcome {
        match self.timer.on_tick() {
            TickOutcome::QuantumElapsed => {
                self.quanta += 1;
                TimerOutcome::QuantumElapsed
            }
            TickOutcome::Tick => TimerOutcome::Tick,
        }
    }

    /// Handle one IRQ0 and, only at a quantum boundary, run
    /// `on_quantum` (the scheduler tick) and return its result.
    /// Returns `None` on a plain tick — `on_quantum` is not called.
    pub fn handle_irq_with<F: FnMut() -> R, R>(
        &mut self,
        mut on_quantum: F,
    ) -> Option<R> {
        if self.handle_irq() == TimerOutcome::QuantumElapsed {
            Some(on_quantum())
        } else {
            None
        }
    }
}

impl Default for TimerIrq {
    /// 100 Hz default: one tick per quantum, matching the scheduler.
    fn default() -> Self {
        Self::new(1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::Cell;

    #[test]
    fn quantum_of_one_calls_tick_on_every_irq() {
        let mut irq = TimerIrq::new(1);
        let calls = Cell::new(0u32);
        for _ in 0..5 {
            let out = irq.handle_irq_with(|| {
                calls.set(calls.get() + 1);
                42u32
            });
            assert_eq!(out, Some(42));
        }
        assert_eq!(calls.get(), 5);
        assert_eq!(irq.ticks(), 5);
        assert_eq!(irq.quanta(), 5);
    }

    #[test]
    fn tick_fires_exactly_at_quantum_boundaries_and_not_before() {
        let mut irq = TimerIrq::new(10);
        let calls = Cell::new(0u32);
        // Ticks 1..9: plain ticks, the callback must NOT run.
        for i in 1..10 {
            assert_eq!(
                irq.handle_irq_with(|| calls.set(calls.get() + 1)),
                None,
                "tick {i} must not call the scheduler"
            );
        }
        assert_eq!(calls.get(), 0);
        assert_eq!(irq.ticks(), 9);
        assert_eq!(irq.quanta(), 0);
        // Tick 10: quantum boundary, exactly one call.
        assert!(irq.handle_irq_with(|| calls.set(calls.get() + 1)).is_some());
        assert_eq!(calls.get(), 1);
        assert_eq!(irq.quanta(), 1);
        // Ticks 11..19: quiet again; tick 20: second call.
        for _ in 0..9 {
            assert_eq!(irq.handle_irq_with(|| calls.set(calls.get() + 1)), None);
        }
        assert_eq!(calls.get(), 1);
        assert!(irq.handle_irq_with(|| calls.set(calls.get() + 1)).is_some());
        assert_eq!(calls.get(), 2);
        assert_eq!(irq.ticks(), 20);
        assert_eq!(irq.quanta(), 2);
    }

    #[test]
    fn handle_irq_reports_outcome_without_callback() {
        let mut irq = TimerIrq::new(2);
        assert_eq!(irq.handle_irq(), TimerOutcome::Tick);
        assert_eq!(irq.handle_irq(), TimerOutcome::QuantumElapsed);
        assert_eq!(irq.handle_irq(), TimerOutcome::Tick);
        assert_eq!(irq.handle_irq(), TimerOutcome::QuantumElapsed);
        assert_eq!(irq.quanta(), 2);
    }

    #[test]
    fn quantum_zero_behaves_as_one() {
        let mut irq = TimerIrq::new(0);
        assert_eq!(irq.quantum_ticks(), 1);
        assert_eq!(irq.handle_irq(), TimerOutcome::QuantumElapsed);
    }
}
