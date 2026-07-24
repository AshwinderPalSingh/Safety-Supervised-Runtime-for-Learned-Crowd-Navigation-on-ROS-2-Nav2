# Transferable engineering lessons

The project-specific bugs and their fixes are documented where they were found
(`docs/phase*-findings.md`, `docs/audit.md`). This document pulls out the general, reusable
lessons - the ones that generalize past this specific ROS 2/Gazebo/crowd-navigation codebase to
almost any real system with simulated ground truth, multiple coordinate frames, or a
config-driven safety layer.

## 1. A component that looks configured correctly can still never actually run

**What happened**: `LOW_PERCEPTION_CONFIDENCE` (Phase 9) was fully implemented, unit-tested,
and enumerated in the trigger taxonomy - and structurally unreachable, because the code path
that would have fed it a nonzero value was never wired up. It looked done. It compiled, it had
a test, the design doc described it correctly. Nothing about reading the code or the docs would
tell you it could never fire.

**The generalizable pattern**: enumeration is not evidence of reachability. A taxonomy, a switch
statement, an if/elif chain, a list of "supported cases" - each entry can be independently
well-implemented and the whole thing still have members that no real input can ever select. The
only way to know is to ask, for each one specifically: *what real state makes this fire, and can
anything in this system currently produce that state?* Not "is the check correct" - "can the
check's precondition ever become true."

**How it generalizes**: this project hit the same shape of bug four more times after the first
one prompted the question. Three more OOD trigger causes (`CROWD_SIZE`, `RELATIVE_SPEED`,
`COMMAND_LIMIT`) turned out unreachable under the actual deployed configuration, even though the
mechanism worked - the *scenario/config values in use* never happened to cross the relevant
threshold. This is a distinct failure mode from a code bug: the code is right, the *environment
it's tested in* just never exercises it. Any system with configurable thresholds and a fixed
test/eval harness should ask this question about every branch, not just the ones that already
looked suspicious.

## 2. Two components can each be individually correct and still be wrong together

**What happened**: `WorldState.robot` (map frame, from Nav2's own pose) and `WorldState.humans`
(world frame, from a ground-truth-reading perception source) were each internally consistent and
correctly implemented in isolation - and combining them into one struct silently mixed two
different coordinate frames. Every relative-distance calculation between them was wrong, for as
long as any live integration test existed.

**The generalizable pattern**: a correctness bug doesn't have to live inside any single function.
It can live entirely at the *seam* between two components, each of which is fine on its own
terms. Unit tests are structurally poor at catching this class of bug, because a unit test
author naturally constructs all the inputs a function needs in one sitting, in one implicit
frame of reference - the very act of hand-building test data erases the distinction between "two
independently-sourced quantities that happen to be in the same frame" and "two quantities in
different frames that a real caller would combine." Confirmed directly in this project: every
test that built a `WorldState` by hand put the robot and the humans in one consistent frame,
because there was no second, independently-sourced pose available to disagree with the first
one inside a plain unit test.

**How it generalizes**: integration boundaries - anywhere two subsystems each supply half of a
computation from their own independent source of truth - need their own dedicated test, one that
deliberately constructs the two sources as genuinely independent (not hand-picked to agree), or
they need to be checked live, adversarially, on purpose. "Both halves are unit-tested" is not
evidence the seam between them is correct.

## 3. A "measurement" can be a tuning value wearing a measurement's name

**What happened** (Phase 2): `caster_mass` was documented and typed exactly like every other
physical measurement in `MEASUREMENTS.md` - a `[PENDING]` value to be replaced once the real
robot was in hand. It wasn't a measurement at all. It was a simulation-stability value found by
trial and error while root-causing a tip-over bug; the real caster almost certainly weighs less,
and substituting the "real" measured value back in would very likely reintroduce the tip-over.

**The generalizable pattern**: a value's *type signature* (a float, in a config file, next to
other measured floats) carries no information about its *provenance* (measured from reality vs.
tuned until a symptom went away). Two values can be syntactically identical and epistemically
completely different, and nothing about the code will tell you which is which unless someone
wrote down why.

**How it generalizes**: any config value that was arrived at by "tried numbers until the
behavior looked right" needs to be documented as exactly that - not folded anonymously into a
table of otherwise-legitimate measurements or otherwise-legitimate hyperparameters - with enough
context that a future person doesn't "correct" it back to something plausible-looking and
reintroduce the bug it was fixed to avoid.

## 4. A plugin's parameters can accurately describe behavior the plugin doesn't actually have

**What happened** (Phase 4, found in Phase 10): Gazebo's `PosePublisher` system plugin was
configured with `publish_model_pose=true`. That parameter exists, is documented, and describes
exactly the intended behavior. On this specific gz-sim version, it never actually published
anything - confirmed by direct `ign topic -e` observation timing out with zero messages, not by
reading the plugin's source or documentation. The dependent topic
(`/ground_truth/robot_pose`) silently carried no data for three phases, and everything downstream
(a perception feature's FOV filter, later an entire evaluation harness) degraded to "no-op" in a
way that produced *plausible-looking* output rather than an error.

**The generalizable pattern**: "I configured the documented parameter correctly" is not the same
claim as "the software does what the parameter says," especially for third-party/vendored
components at a specific pinned version. This is a much stronger case for direct verification
than code you wrote yourself, because you have even less visibility into why it might not work.

**How it generalizes**: for any dependency whose correct operation you're relying on but didn't
write, verify the *actual data path* end to end (does the topic/queue/callback really carry
real values, checked with an external observer, not just "did the configure/subscribe call
return without error) at least once, especially right after adopting it, rather than trusting
config-level correctness to imply behavioral correctness.

## 5. The same bug class can recur even after you've fixed it once, and even in your own new code

**What happened**: a script losing its executable bit broke a launch four separate times across
this project - twice in Phase 10 (different scripts), once discovered by a systematic sweep
during Phase 11, and once in a file written *during this same audit's own tooling*, days after
the pattern had already been identified and supposedly closed.

**The generalizable pattern**: fixing the instance you found is not the same as closing the
class. A bug that's mechanical and cheap to reintroduce (a chmod, a copy-paste, a git operation
that silently drops a mode bit) will keep recurring precisely because nothing about writing new
code reminds you to check for it - each new occurrence looks like an unrelated, one-off surprise
unless you're specifically checking the *category*, not just the specific file that broke last
time.

**How it generalizes**: when a bug's root cause is "a property that's easy to lose and expensive
to notice is missing" (an executable bit, a config value staying in sync across two files, a
frame label on a coordinate), the fix that actually closes it is a blanket sweep or an automated
check across every instance of that category - not a point fix to the specific file that
happened to break. This project's own git-mode sweep (checking *every* script's tracked index
mode, not just the ones already known to have broken) is the concrete version of this lesson,
and it found a fourth instance the moment it was actually applied broadly instead of reactively.

## 6. A cleanup/safety mechanism can turn into the exact hazard it exists to prevent

**What happened**: a stray-process sweep, built specifically to kill leftover Gazebo processes
between evaluation episodes, used a broad `pkill -f` pattern list borrowed from an existing,
already-reviewed script. That pattern list matched a substring of the *calling* process's own
command line (which embedded a JSON-serialized config blob containing the literal text
`"nav2_mppi_controller"`), and the safety sweep killed the harness process out from under itself
- for one specific configuration, silently, every time.

**The generalizable pattern**: a broad text-matching cleanup mechanism (`pkill -f`, a glob
delete, a "kill anything matching this pattern" script) is scoped to *the literal text it
matches*, not to *the intent behind why you wrote the pattern*. If anything in the system's own
operation - including the caller's own arguments, environment, or serialized state - can contain
that same text, the mechanism will eventually match itself. This is worse than an ordinary bug,
because the mechanism's entire job is safety/cleanup, so its own failure mode is silent
self-destruction, not a loud error.

**How it generalizes**: any broad pattern-based cleanup/kill/delete mechanism should be checked
against the *full space of things that could legitimately contain that pattern* - not just "does
it match what I intend to kill," but "what else, including my own process's own command line or
data, might also match, and what happens if it does." Narrow, purpose-specific patterns (checked
against what the calling context can actually contain) are safer than reusing a broad,
general-purpose pattern list in a new context, even one that's already been reviewed and trusted
elsewhere.

## 7. When auditing your own work for methodology errors, apply the same rigor to the audit itself

**What happened**: re-verifying this project's own "seeded simulation is byte-identical"
determinism claim, during this same audit, produced two false "the claim is wrong" results in a
row - both traced to flaws in the *verification script*, not the system under test (comparing by
raw message-arrival index instead of simulated-time key; reusing one continuously-running clock
source across two supposedly-independent trials). Only a third, more careful attempt, matching
the original methodology's own alignment strategy, confirmed the claim actually holds.

**The generalizable pattern**: an audit or verification pass is itself code, with its own bugs,
and a rushed verification script is exactly as capable of producing a false positive as any other
rushed code. The standing instruction not to let a real finding get reframed as "defensible under
a different reading" cuts both ways - a *false* finding dismissed too quickly, without the same
scrutiny given to a real one, would be exactly as dishonest as softening a real one.

**How it generalizes**: when a verification check disagrees with an existing, previously-
validated claim, the first hypothesis should be "is my new check wrong," checked with the same
rigor as "is the original claim wrong" - not settled by picking whichever answer requires less
follow-up work.
