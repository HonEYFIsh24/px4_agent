# Native VINS stationary health experiment

## V4 measured-time keyframes and bias-chain diagnosis

Shadow flight 05 completed the physical route and the full 30-sim-second
postland observation, with no native reset/adapter epoch change and only
4.02 mm postland error change. It still fails the unchanged accuracy gate:
position RMSE 0.283600 m exceeds 0.25 m. The terminal single-initial-yaw-and-
translation-aligned error was [-0.07481, +0.33487, +0.09044] m. Relative
roll/pitch error evolved from about [-0.879, +0.788] degrees initially to
[+5.998, +4.212] degrees at the end. Read-only phase and orientation diagnostics
are in `logs/ev/shadow_flight_05/native_position_error_analysis_01.{json,md}`.
Ground truth was used only for evaluation, never for estimation or control.

The official non-keyframe `MARGIN_SECOND_NEW` code appends every actual IMU
step of the latest interval to `pre_integrations[WINDOW_SIZE-1]`. During
prolonged low-parallax stillness this merged interval grows without a bound.
The ordinary optimizer skips an IMU factor when its `sum_dt > 10 s`; OLD
marginalization retains its first IMU bridge only when `sum_dt < 10 s`
(the exact 10-second boundary differs). That omission removes the direct
IMU/bias random-walk bridge that would transmit the one initial bias prior.
Visual pose factors may still connect the overall nonlinear graph. ZUPT and
relative position do not themselves supply this missing bias bridge. The
old flight-05 collector lacks window span data, so this source mechanism
and deterministic regression are not claimed as observed runtime proof
of that particular flight's entire error.

V4 adds optional `max_imu_preintegration_span_s`. Native default 0 leaves
the original keyframe policy enabled; only the generated SITL configuration
sets 1.0 second. Valid nonzero values are 0.2–5.0 seconds. Before the current
optimization, for IMU-enabled full windows including initial stereo startup,
the actual prospective merged span is the sum of the two retained native
preintegration spans. If that sum reaches the configured bound, the current
image is retained with standard `MARGIN_OLD`, even when visual parallax is
small. The existing visual keyframe decision continues to work independently.
All raw IMU steps, real dt, normal OLD marginalization, pose/velocity/bias
optimization and stationary interval flags are preserved. No dt is clipped,
no timestamp rewritten, no state frozen, no repeated zero-bias prior added,
and no absolute orientation is pinned. An actual single source interval
longer than the bound remains long and visible; this policy prevents
non-keyframe accumulation, not real sensor outages.

`native_health` now identifies `stationary_health_v4`. `imu_optimizer_window`
is an explicit **before-solve-and-slide** snapshot of all camera source stamps,
each actual integrated IMU duration and original step count, optimizer
included/skipped edges, OLD bridge inclusion, visual/time keyframe reasons,
and existing prior SpeedBias parameter membership. Its topology reachability
field is not proof of initial-bias information rank: a SpeedBias block could
contain only velocity information. `prior_information_rank_inferred=false`
makes that limit explicit. The **after-process-image-and-slide** current state
has its own real source stamp and pose, quaternion, velocity, Ba and Bg. Raw
last IMU sample stamp/vectors and the one-second window's actual first/last
stamps, vector means and per-axis standard deviations are also recorded.
Ceres iteration counts, successful/unsuccessful steps, termination enum,
elapsed wall time, configured time budget and last gradient magnitude expose
budget-limited usable solutions. The native source keeps the upstream option
handling; the separate SITL budget experiment below changes only its generated
configuration, with all measurement timestamps and acceptance gates preserved.

`tests/vins_imu_chain_test.cpp` runs a deterministic 30-second replay of the
actual OLD/SECOND_NEW slot logic. It demonstrates the disabled policy's
longer-than-10-second omitted link; with the 1-second policy all active
links stay bounded under the tested 100-ms image cadence and the direct
prior path remains continuous. It also checks mixed visual keyframes and
every 4-ms original step appearing exactly once in active or marginalized
history. Separately it compiles the actual native `MarginalizationInfo` /
`MarginalizationFactor`, starts with the one real `ImuBiasPrior`, performs
30 standard Schur eliminations with bias random-walk bridges, verifies
retained information `1/(initial_variance + sum(process_variance))`, and
solves the retained factor with real Ceres. This confirms information
retention in a linear bias subproblem, not full nonlinear VIO observability.
Fresh V4 application and V1/V2/V3 upgrades are reproducible and reject
partially patched sources.

The existing initial gyro calibration was also read: `solveGyroscopeBias`
uses visual relative orientation and each genuine preintegration Jacobian,
updates all Bg states, and repropagates both initial image and estimator IMU
integrations with that bias. `double2vector` fixes normal world yaw/translation,
rotates world velocity, and correctly leaves body-frame biases unrotated.
Its full-rotation singularity fallback requires pitch near 90 degrees and
does not explain the small-pitch flight-05 case. No definite prior-address
shift bug was found in standard OLD/SECOND_NEW mappings. A later ground
snapshot has gyro mean minus estimated Bg about [-0.00010,+0.00019,-0.00010]
rad/s, comparable to the mean's measurement uncertainty, while lateral
accelerometer bias and tilt are strongly inconsistent with physical truth.
It does not justify adding a new gyro constraint without timestamped
within-flight bias evidence. Shared-midpoint Q correlations, OU mean reversion,
nonlinear bias/gravity observability and Ceres budget convergence remain
explicit model/evidence questions. Flight 06 must still pass the original
full-flight RMSE/max-error/timing and postland requirements.

The local `stationary_health.patch` extends the official VINS-Fusion estimator.
`vio/patch_upstream.py` checks and applies the diff after the existing native
health and initial IMU bias patches. `stationary_factors.h` is its reproducible
factor/detector source; a second application is a no-op. A partial patch aborts
building. No Gazebo truth, GNSS, EKF position or vehicle landed flag enters the
estimator or the stationarity detector.

## Evidence that can enable ZUPT

The original body IMU must provide a one-second window with at least 100
samples, at least 0.98 seconds of span, and sample age between -0.005 and 0.02
seconds relative to the image plus calibrated VINS time offset. Raw gyro mean
norm must be at most 0.01 rad/s, gyro standard-deviation norm at most 0.008
rad/s, acceleration standard-deviation norm at most 0.07 m/s², and mean
acceleration norm must be within 0.3 m/s² of configured gravity. These are
sensor-motion gates, rather than navigation state or a confidence probability.
The acceleration threshold allows the simulated original IMU's approximately
0.029 m/s² noise on each of three axes (norm approximately 0.051).

At least 20 feature IDs must remain real temporal tracks in both stereo eyes,
with positive left-minus-right disparity, vertical disparity at most 1.5
pixels and calibrated depth `fx * baseline / disparity <= 8 m` at every
measurement. The cameras must be rectified with a positive optical x baseline.
New, missing or reconstructed IDs lose their histories immediately. No zero
velocity assigned by the feature tracker to a new feature can establish
stationarity. At least eight actual observations spanning at least 0.98 s of
a one-second window are required for each retained ID. Windows containing a
missing image interval over 0.2 s are discarded.

Each eye's raw pixel positions are fit against real source timestamps using
least squares. The faster fitted eye is used per feature. Median fitted speed
must be at most 0.25 px/s and the 90th percentile at most 0.5 px/s; these
physical motion limits are unchanged. The 90th percentile of two-eye fit
residual RMS must be at most 0.06 pixels. Instantaneous actual normalized
feature velocities, converted using the true 320-pixel focal length, remain
a motion-release channel: p90 above 2 px/s or median above 0.75 px/s clears
support at that frame. Original IMU motion/freshness gates also clear support
immediately. A complete stereo/IMU gate must then persist for an additional
second before any ZUPT residual is enabled.

The 15-sim-second ground calibration `logs/ev/ground_tracks_01` supplied 153
actual native frames. Its 143 complete one-second visual windows retained
68–74 nearby two-eye tracks. Fitted p90 speed ranged 0.0498–0.1271 px/s, fitted
median speed stayed below 0.02484 px/s, and p90 fit jitter stayed below
0.03050 pixels. Instantaneous p90 reached 0.7483 px/s and instantaneous median
0.2653 px/s. The 0.06-pixel jitter gate allows about twice the observed maximum,
while the short-motion gates allow at least 2.6 times the observed ground
maximum. All 143 complete windows meet the proposed visual gate. This
calibrates image measurement noise; it does not prove post-flight ground
tracking or arbitrary real environments. The standalone analyzer is
`vio/analyze_stereo_tracks.py`, and its immutable result is
`logs/ev/ground_tracks_01/native_track_calibration.json`.

These conservative conditions intentionally withhold ZUPT when nearby
stereo tracks are insufficient. Far scenery and uniform-speed motion cannot
be declared stationary from an IMU gravity-norm check. The detector assumes
a sufficiently static nearby scene. Very slow motion below the optical-flow
threshold remains a possible ambiguity; this experiment does not establish
stationarity for arbitrary real scenes or replace flight validation.

## Actual estimator constraints and failure handling

`enable_zupt` defaults to 0 in native code. The generated simulation experiment
configuration uses 1, velocity sigma 0.03 m/s and relative-position sigma
0.02 m. Every image/window state has its own stationary flag. Only those states
receive zero-velocity residuals. Relative-position residuals join adjacent
states only when both carry independently confirmed support and their whole
retained interval has stationary support. A separate interval flag follows
state shifts. Removing a non-keyframe combines both adjoining interval flags
with AND, while also checking all three endpoint/middle-frame flags. Dropping
a moving or unconfirmed middle frame cannot connect two stationary endpoints
with a stationary constraint. The surviving current frame keeps its own
zero-velocity evidence. Appropriate old constraints are marginalized. The current stationary state cannot impose zero velocity on a
past moving state. Moving frames receive no new stationary residual. These
are soft optimization residuals: published poses are neither frozen nor
moved toward any absolute-position reference.

Each Ceres solve must return `Summary.IsSolutionUsable()`, finite nonnegative
costs with no increase, nonempty finite residuals and finite Jacobians. Hitting
the configured iteration/time budget can still be a usable solution. New
marginalization priors must be valid and finite. Optimized position,
orientation, velocity and biases must also be finite. Consecutive optimized
states retain the adapter's existing position jump bound `max(0.35, 4*dt)` m
and attitude bound `max(0.25, 6*dt)` radians. The additional difference between
pose displacement and trapezoidal velocity integration must be at most
`0.35 + 3*dt²` m, allowing ordinary optimization corrections while rejecting
the observed landing drift/jump. Failed results are not published or used to
refresh IMU propagation. Native tracking becomes false and VINS resets once;
the EV adapter then requires explicit safe ground reanchoring.

`/vins_estimator/native_health` publishes measurement-time JSON with actual
solver costs/residual RMS and the raw stationarity evidence. This diagnostic
is not a calibrated uncertainty or tracking probability. The configured EV
covariance remains a separate conservative model.

## Validation and remaining gate

`tests/vins_stationary_factors_test.cpp` compiles against the installed real
Ceres library and exercises raw IMU windows, support timing, positive/near
stereo tracks, motion rejection, analytic factor Jacobians, a real optimization,
selective per-state constraints, actual OLD/SECOND_NEW slide helpers (including
repeated non-keyframe removal across motion), release across motion and the actual earlier
landing failure values. `tests/test_vins_stationary.py` builds/runs that test
and verifies reproducible/idempotent patch application and rejection of a
partial patch. The actual Noetic `vins_node` workspace also builds.

Runtime acceptance requires a complete shadow flight followed by at least
30 simulated seconds on the ground, with source timestamps, raw VINS,
adapter epochs, native health and independent ground-truth evaluation saved.
Compilation and factor tests alone do not prove the landing defect resolved.
The earlier first-flight recorder stopped before the real landing jump and
therefore cannot count as a passing post-landing window. GPS fusion and GPS
denied flight remain gated on this complete shadow validation.


The V2 patch fixes the V1 non-keyframe interval-continuity limitation. The
installer supports a checked, reviewable V1-to-V2 upgrade and full fresh
application. Shadow flight 04 ran the V1 process and must retain that identity:
its adapter rejected a real 0.35058 m optimized-to-propagated refresh jump at
sim 48.276 seconds. No native stationary frame had been confirmed before
that failure, so the interval-continuity defect cannot explain that run's
first jump. It remains a separate optimizer/propagation consistency failure,
with no acceptance-threshold relaxation.


## Original IMU discrete covariance calibration

The Iris original IMU runs at nominal dt=0.004 s. Its plugin produces each
white-noise sample with standard deviation `noise_density/sqrt(dt)`. Native
VINS `integration_base.h` directly squares `ACC_N/GYR_N` into per-sample noise,
and its bias-step Jacobian multiplies `ACC_W/GYR_W` by dt. The simulation
configuration and generator now use the explicit one-step convention:

- acceleration N = 0.00186/sqrt(0.004) = 0.029409182239566 m/s² per sample;
- gyro N = 0.00018665/sqrt(0.004) = 0.0029511956263521 rad/s per sample;
- acceleration W = exact OU step sigma / dt = 0.094867697353033;
- gyro W = exact OU step sigma / dt = 0.00061324346876081.

The OU step sigma is `q*sqrt(tau/2*(1-exp(-2*dt/tau)))`, using q=0.006,
tau=300 s for acceleration, and q=0.000038785, tau=1000 s for gyro. Turn-on
bias sigmas (0.196 m/s², 0.0087 rad/s) remain their separate initialization
prior. The original ROS IMU plugin forwards the measurements without adding
noise or taking navigation-state input. Its published covariance already
uses density²/dt.

This fixes the old continuous/discrete parameter mismatch. It does not
establish full covariance consistency: native preintegration omits OU mean
reversion and cross-step correlation from reused midpoint samples. A sqrt(2)
white-noise correction for long integrations has not been silently applied
or declared validated. Ground native health from shadow 04 measured IMU
acceleration standard-deviation norm 0.0503–0.0541 m/s² before flight, matching
the source-model baseline sqrt(3)*0.029409 ≈ 0.05094. The detector's 0.07
variance gate is calibrated to this actual source noise; flight error/jump
acceptance limits remain unchanged.

Native raw stereo coordinates and normalized temporal velocities are included
in `native_health.stereo_tracks` for sensor-window calibration. Per-run output
paths use `UAV_VIO_LOG_DIR` when set, falling back to `logs/vio`; pose graph
output follows the same directory. Native CSV evidence is no longer overwritten
by the next manager run when the manager supplies its owned directory.


## V3 propagation refresh and reset concurrency

Before `updateLatestStates` commits any refreshed prediction, a candidate is
repropagated using actual buffered original IMU timestamps. It is compared
with the previously published trajectory at exactly the same real source
stamp. When a queued IMU callback is waiting for the propagation mutex, that
same real sample can advance the old prediction to the comparison time.
Receipt time never substitutes for a measurement timestamp. Both native IMU
and configured physical body-center positions are checked against the
existing 0.35 m jump floor, and attitude against 0.25 rad, with only 1 µm /
1 µrad of numerical tolerance. The configured IMU-to-body center offset
`[0,0,-0.02]` m handles the genuine reference-point difference. A bad candidate
is rejected; no clipping, frozen poses or automatic frame reanchoring occurs.

Refresh failure sets the atomic solver state to INITIAL while still holding
`mPropagate`; a waiting IMU callback rechecks that state under the same lock
before publishing. Callbacks cannot rewind to an older stamp. Queue clears,
feature snapshots and IMU availability reads have `mBuf` protection. Native
IMU initialization/integration and image processing run under `mProcess`,
with reset snapshots revalidated after waiting for IMU. The waiting loop does
not hold `mProcess`, so ROS restart cannot block IMU delivery behind it.
Existing lock order is process -> propagation -> buffer, while input IMU
releases the buffer lock before taking the propagation lock.

The V3 process reported `implementation_revision=stationary_health_v3`,
actual instantaneous and fitted flows, complete-window track count/span,
window-fit jitter and measured refresh deltas, together with raw stereo
tracks. V3 is reproducible from fresh source, V1 or V2. The final flight still
requires the full post-landing 30-sim-second window under unchanged acceptance
limits; the initial ground calibration does not replace that gate.

## SITL solver-budget experiment after shadow 06

The actual `shadow_flight_06` record contains 661 optimization-health samples:
620 report Ceres `NO_CONVERGENCE`, 41 report `CONVERGENCE`, and 638 finish beyond
their configured 0.032/0.040 wall-second limits. These were usable solutions,
and the original full-flight quality gate passed at 0.153116 m position RMSE
with 3.223 mm maximum drift in the real 30-sim-second post-landing window.
Passing this position gate does not establish absolute attitude accuracy.
The following `gps_fused_flight_01` startup produced approximately 10 degrees
of ground tilt and large accelerometer bias; its mission failed and remains
recorded as failed. Budget shortage is a plausible contributor rather than a
proven sole cause of that tilt or the later visual-track reset.

For the next controlled SITL run, only `max_solver_time` becomes **0.12 wall
seconds** and `max_num_iterations` becomes **12** in `stereo_imu.yaml` and the
asset generator. Upstream source still applies its 80% allowance to OLD solves,
so those receive 0.096 wall seconds. Ceres iteration diagnostics include the
initial iteration; a configured maximum of 12 can therefore yield 13 entries.
At simulation speed 0.35, ten actual image optimizations per simulation second
allow approximately 0.286 wall seconds per image. A nominal 0.12-second solve
leaves 0.166 seconds for the rest of the pipeline; this is a capacity estimate,
not evidence that all processing fits or that Ceres will converge. Its time
limit can be exceeded by work already in progress during an iteration.

The next manager must first record 15 actual ground simulation seconds and
check observed iteration/termination counts, optimization throughput, original
measurement age, producer timing, native bias/tilt and propagation refresh.
The full route and real 30-sim-second post-landing gate remain required. No
source timestamp, sensor-noise model, prior, stationary detector, covariance,
frame-jump guard, scale, truth feedback, simulation speed or quality threshold
changes as part of this experiment. Existing processes keep the options they
loaded at startup; updating the files does not change a running estimator.

### V5: cold-start bias information diagnostics (opt-in only)

The larger `.12/12` solver budget improved ground convergence (137/151
CONVERGENCE) but the actual 15-sim-second ground capture still had
11.27–11.53 degrees of gravity tilt and accelerometer bias near 1.7 m/s².
Its position/timing stability does not validate attitude or establish a cause.
The next run therefore records evidence without adding estimator constraints.

Set `PX4_AGENT_VINS_BIAS_DIAGNOSTICS_DIR` to an exclusive absolute directory
before starting the owned manager. Empty/unset keeps diagnostics disabled.
The generated YAML uses actual camera state source stamps: every optimization
for the first three simulation seconds after the first full window, then no
more often than once per simulation second. Actual Ceres variable-only CRS
Jacobians, robust residuals/gradient/cost, explicit parameter order and local
basis, residual segment RTTI/row counts, and global state values are dumped
before solve, after solve and after native yaw gauge restoration. OLD and
SECOND_NEW marginalization dumps include their actual incoming factor stack,
outgoing prior, stored linearization values, mapped fixed-calibration flags,
and the native already-computed Amm/Schur eigenvalues. The first OLD is retained
at full cadence. Files are exclusive; the writer refuses existing paths.

No SVD runs in the estimator. Offline analysis must exclude actual constant
calibration columns before eliminating nuisance variables and must report both
unscaled and column-normalized nuisance SVD with threshold sensitivity. Tiny
numerical global-gauge leakage can produce false rank loss when amplified by
column normalization. Bias rank alone is insufficient: compare effective
gradient/center, original residual and source-state mapping, and the four
translation/yaw gauge null directions. The constant-IMU/stereo independent
fixture preserved one bias prior through ten OLD steps and did not reproduce
the actual startup tilt. It is a useful negative case, not runtime validation.

This version also corrects upstream `delete` to `delete[]` for arrays allocated
by `preMarginalize`. The ownership fix is independent of the tilt hypothesis;
no evidence currently attributes the observed tilt to that undefined behavior.
The narrow native regression exercises actual IMU/pose nuisance elimination,
once-only prior, OLD transfer, effective gradient, fixed-calibration semantics
and exclusive JSON serialization. A standalone AddressSanitizer case executes
100 actual native allocation/destruction cycles. V1–V4 migrations, clean V5
installation and idempotence remain checked against the final native sources.

Opt-in disk serialization adds wall-time overhead. Its cold-start capture is a
diagnostic experiment; source timestamps, loss functions, constraints, health
gates and flight acceptance limits remain unchanged. An early recorder must
attach before tracking initializes and retain actual raw IMU/native/clock/EV
producer timing. Ground diagnostics cannot substitute for a full route and
actual 30-sim-second post-landing evaluation.

### V6: experimental exact derivative of the represented frozen prior

The cold V5 matrices establish a numerical inconsistency independent of the
unproven 11-degree startup cause. At actual third OLD (source 8.152 s), the
represented prior's global-yaw objective derivative was −71.550750 by finite
differences, while the native frozen Jacobian declared +13.198783. A finite
native yaw restoration also changed the represented prior cost while IMU and
visual costs remained invariant. First/second OLD did not show clear Ba prior
loss. These are distinct findings; finite FEJ yaw noninvariance remains after
an exact derivative repair.

`exact_marginalization_pose_jacobian` defaults to **0** in upstream-compatible
native parameter loading. The generated SITL configuration sets it to **1**
for a controlled experiment. Every `MarginalizationInfo` captures the value as
an immutable `const bool`; both OLD and SECOND_NEW carry this mode. Native
health reports both the loaded `marginalization_pose_jacobian_mode` and the
actual `active_prior_pose_jacobian_mode` (`none` before a valid prior), even
with all matrix diagnostics disabled.

For the existing residual rotation chart `2 * sign(q_rel.w) * q_rel.vec`,
where `q_rel = q0.inverse() * q`, its exact right-body tangent derivative is
`sign(q_rel.w) * (q_rel.w * I + skew(q_rel.vec))`. Only the returned rotation
Jacobian columns multiply the frozen prior's rotation columns by this matrix.
The frozen prior matrix, residual offset, linearization point, represented
residual, translation and speed/bias derivatives are unchanged. Native factors
already embed body-tangent columns in the first six columns of a global-seven
pose block; the existing `PoseLocalParameterization` convention is retained.
No pose/gauge/roll/pitch constraint or repeated bias prior is added.

At a non-cut rotation this derivative is an invertible change of pose-nuisance
coordinates, so it adds no prior information. At exactly 180 degrees the
existing hemisphere chart is nondifferentiable; this change does not smooth
or omit the factor there. Tests use finite differences inside the ±179-degree
hemispheres and explicitly retain the residual jump across the 180-degree cut.

The permanent C++ tests replay compressed actual second/third OLD frozen
priors and current states (source hashes embedded in the fixture), compare
actual native PosePlus residual/Jacobian and objective finite differences at
three step sizes, verify quaternion sign equivalence and bit-identical legacy
mode, and check the invertible nuisance-basis relation. The exact third-OLD
derivative agrees with finite differences within 1.3e-6 absolute; whole local
Jacobian relative error is below 5.1e-10. An actual-factor constant-measurement
fixture corrects a compensated 11.454-degree initial tilt to 1.342 degrees
with all roll/pitch coordinates free and one bias prior carried through ten
OLD eliminations. A separate dynamic stereo/IMU case observes 0.200046 radians
of relative heading for a 0.2-radian rotation with six free pose coordinates.
Those tests validate math/behavior, not production startup repeatability.

Diagnostic schema 2 separates `prior_before_solve`'s evaluated current Jacobian
from `stored_frozen_prior_jacobian` and keeps the frozen residual/linearization
values. Its local coordinate basis remains unchanged. The prior mode is
explicit; an exact current Jacobian must not be replayed as the frozen matrix
in `r0 + J0 * dx`. Original V5 artifacts remain unchanged.

Before flight, the predeclared experiment requires three fresh, independent,
matrix-diagnostics-OFF cold ground boots with 30 actual simulation seconds.
All must pass the original position/timing gates and the declared six-degree
gravity-orientation limit. Stop on the first failure; failed starts remain
recorded. No acceptance threshold, simulation speed, measurement timestamp,
source freshness, covariance or native/adapter jump/reset guard changes.

### V7: experimental first-pose translation/yaw coordinate gauge

The first V6 predeclared diagnostics-OFF cold boot failed the independent
six-degree gravity gate (maximum 6.311357 degrees). Its 303 late-ground native
samples all had confirmed sensor-supported stationary constraints, while tilt
and compensating accelerometer bias grew. Position and timing passed. Exact
prior derivatives did not establish startup repeatability; the failed start
remains recorded and the planned sequence stopped without another seed.

`fix_first_pose_xyz_yaw` defaults to **0** in native loading. The generated
SITL experiment sets it to **1**, retaining V6's exact represented-prior
Jacobian. Only Ceres `pose_0` changes from global7/local6 to global7/local2.
Its current XYZ and ZYX Euler yaw define the coordinates; roll and pitch remain
free. Every later pose retains six free local coordinates. There is no extra
residual, absolute roll/pitch constraint, repeated bias prior, GPS input or
truth feedback. Upstream `MarginalizationInfo::localSize(7)` remains **6**;
OLD and SECOND_NEW continue eliminating physical native pose tangents.

For `R = Rz(yaw) Ry(pitch) Rx(roll)`, the right-body tangent map from Euler RP
has rotation columns `[1,0,0]` and `[0,cos(roll),-sin(roll)]`, with zero
translation rows. Both local coordinates use **milliradians**, a fixed
`0.001 rad/local-unit` conversion: `Plus` applies this same scale and every
factor receives `Jphysical6 * B6x2 * 0.001`. This units choice is explicit
because Ceres evaluates `Plus(x,-raw_gradient)` for its projected gradient norm
before deciding a trust-region step. The unscaled candidate failed an actual
wrong-tilt solve when that probe left the Euler chart; its failure logs remain
preserved. A fixed scale changes numerical conditioning and Ceres's reported
gradient norm; it must not be interpreted as proof of physical convergence.
No scale is chosen from truth or adjusted using the current gradient. An
arbitrarily large raw-gradient probe can still fail for any finite scale;
this experiment does not establish unconditional solver robustness. The
negative accelerated-motion case that initialized both frames frozen is
preserved; the passing moving case initializes its new pose/velocity from the
same original `IntegrationBase` IMU samples, then verifies stereo corrects
that prediction and the later XYZ/RP/yaw degrees of freedom remain available.
The Ceres projected-gradient behavior follows its
[official trust-region implementation](https://github.com/ceres-solver/ceres-solver/blob/1.14.0/internal/ceres/trust_region_minimizer.cc#L238-L254).

`Plus(x,0)` preserves the same quaternion sign and physical pose, including
wrapped yaw near +/-179 degrees. Input XYZ/yaw outside the captured manifold,
non-unit/nonfinite states and unsupported pitch charts (within one degree of
+/-90) fail. Trial steps outside the supported chart also fail. There is no
roll/pitch clipping, output freezing or alternative gauge fallback. Solver
unusable/chart failures follow the existing tracking-false-before-reset path.

Diagnostic schema **3** describes each matrix block's actual local basis.
Full Ceres CRS matrices use the real local2 Euler-RP milliradian coordinates
for pose0 and local6 native body tangents for later poses. Dense prior and
marginalization matrices remain physical6; each block records
`actual_optimizer_local_size`, `optimizer_to_matrix_local`, and
`physical_pose_tangent_from_optimizer`, so nuisance projection can apply the
actual map instead of treating all physical columns as free. An anchored full
problem has zero remaining global translation/yaw gauge DOFs; the four-DOF
physical gauge quotient is explicitly **not applicable** there. Existing V5
and V6 artifacts are not rewritten. Matrix diagnostics remain OFF ordinarily.

Normal native health records the loaded gauge mode and the actual computed
native yaw/translation restoration delta, optimizer source stamp and local
coordinate units. These measurements are retained when the flag is OFF, too;
zero is never inferred merely from enabling the gauge.

Permanent actual-factor tests cover signed/wrapped `Plus`, its right-body
basis, invalid manifold/singular rejection, captured exact-prior RP2 finite
differences, full IMU/stereo/prior RP2 finite differences, a compensated wrong
initial tilt with RP free through ten OLD steps, and later-pose relative
translation/yaw/RP motion. Synthetic measurements validate solver behavior,
not actual cold-start quality. The next physical experiment still requires
three predeclared fresh diagnostics-OFF 30-sim-second ground boots, stopping
on the first failure, with all original gates and the six-degree limit intact.

### V8: bounded smooth update for actual Ceres gradient probes

V7's first fresh diagnostics-OFF cold boot failed and stopped its predeclared
sequence. Actual native logs contain three `Plus(x,-gradient)` failures:
receipt sim5.804/6.896 (zero iterations) and12.032 (ten iterations). After
native reset, the adapter correctly kept epoch1 latched and forwarded no new
estimate. Later healthy tracking is not a pass or another accepted start.
The failure branch and raw native gradient were not recorded; this run
establishes the solver integration failure, not a new bias-prior rank cause.
The complete V7 source/config/tests/library/node and negative evidence are
archived under `logs/ev/native_v8_gauge_design_01/source_v7_before`.

V8 retains the opt-in gauge/defaults, milliradian units, actual Jacobian sB,
physical marginalization local6 and existing supported |pitch|<89deg input
chart. It replaces only the gauge's update retraction. With T=89deg, s=.001:

    h = (T-theta)*(T+theta)/(8*T)
    theta_next = theta + h*tanh(s*delta_pitch/h)
    roll_next = roll + 2*atan(s*delta_roll/2)

The product form is the smooth quadratic (T²-theta²)/(8T), with no subtractive
cancellation or abs/min seam. Its step magnitude is at most one quarter of
the closest pitch-boundary distance, giving a floating-point margin for a
large probe. The roll increment has magnitude at most pi, preventing the
previous linear Euler update's exact full-turn projected-norm alias. Both
zero-step derivatives remain s, so Jphysical6*sB and schema3 mappings remain
unchanged. XYZ/yaw remain fixed and RP remain two free directions; multiple
steps reach supported pitches and arbitrary physical roll. No physical RP
measurement, prior, clipping, invalid-input projection or frozen output is
added. Zero update copies the original quaternion exactly, retaining its sign.

Finite huge delta/h may overflow to infinity, whose tanh limit is defined and
finite; nonfinite source or delta still fails. Pure C++ tests exercise signed
quaternions/yaw±179, pitch through 89deg−1e−12, every combined ±1e308/DBLmax
probe, invalid input and real custom-Plus/body-Jacobian finite differences.
Near the very closest representable pitch boundary, a sub-ulp update can
round to the same orientation; no claim of nonzero machine step or numerical
rank for every finite gradient is made. The input chart and independent
six-degree ground gate are not narrowed or relaxed.

Ceres's `solver_last_gradient_max_norm` remains its mixed-block projected
ambient step norm. Bounded updates change that algorithm metric; it is not
the physical Euler-RP gradient and does not establish physical convergence.
Normal health marks `solver_gradient_norm_semantics`, actual retraction mode
and source-stamped before/after RP/yaw/XYZ restoration. It does not divide the
mixed summary by s or add per-frame Jacobian recomputation. Only an explicitly
evaluated actual pose0 local2 objective-gradient vector could legitimately be
divided by s to obtain g_RP at that source state.

The original V7 negative moving-frame case is rerun unchanged with both
frames initially frozen. V8 now solves it with actual IMU/stereo/once-bias
factors and a nonzero later XYZ/RP/yaw motion (relativeyaw.200537rad), while
the original IMU-predicted seed also passes (.199769rad). The wrong11.454deg
fixture remains1.34392deg after ten OLD steps with freeRP and one bias prior.
Captured second/third OLD exact-prior cost/J FD and full Ceres J6*sB FD still
pass. These synthetic/frozen tests prove the tested numerical behavior,
not successful cold startup, repeatability or flight. All original V7 failure
and intermediate negative logs remain intact; root exclusively runs the new
three-fresh-boot predeclared V8 cold sequence, stopping on the first failure.

### V9: sensor-supported full-window stereo/IMU initialization

V8 fixed the Ceres gradient-probe domain failures, but its first fresh cold
start still failed independent gravity orientation quality: maximum body-Z
error 12.236165 degrees against the fixed six-degree limit. There was no
solver/reset/epoch failure, and actual EV position/velocity fusion remained
active. The early approximate CSV tilt changed from 1.00 degrees at source
6.548 to 12.13 degrees at 6.748, before the first observed static confirmation
at 15.248. This proves an early wrong solution and later gravity/bias
compensation; it does not prove loss of the initial bias prior, or establish
which first-OLD transition caused the jump. Original source, library, node,
configuration and FAIL evidence are preserved under
`logs/ev/native_v9_initialization_design_01/source_v8_before` and
`logs/ev/native_v8_cold_sequence_01`.

`stationary_initialization` defaults to **0** in native upstream code. The
owned SITL configuration explicitly sets it to **1**, and requires stereo,
IMU and the existing sensor ZUPT detector. While INITIAL, it requires every
actual camera-window state `[0..10]` and each continuous interval `[1..10]`
to have independently confirmed source stationarity, including the current
source gate. This gate uses the existing original IMU variance/mean checks
and tracked, near, positive-disparity stereo temporal-motion checks. Neither
estimated velocity, GNSS, simulator truth nor FCU landed status grants
stationarity. Motion, missing/young source support or a broken interval keeps
INITIAL with no optimization, marginalized prior, optimized odometry or IMU
propagation output. Complete waiting windows always slide OLD.

At the one permitted seed, actual original IMU samples covering the complete
camera source window set gyro bias to their mean, acceleration bias to the
existing factory prior mean zero, and velocities to zero as initial guesses.
A common world rotation aligns the initial gravity direction from the raw
acceleration mean while retaining coordinate yaw; it rotates all poses about
the existing first position and preserves relative visual geometry. It does
not replace every pose orientation by one fixed attitude. All original
window, ImageFrame and current temporary IMU preintegrations are repropagated
with the new bias guesses; sample buffers, timestamps and measured IMU
values are unchanged. Existing roll/pitch variables, the once-only initial
bias prior and the existing IMU, stereo and stationary factors remain free.
No new physical residual, attitude pin, recurring prior or truth correction
is added. The first actual joint solve forces OLD so the existing once-only
bias prior is carried through normal marginalization.

Delayed initialization exposes an upstream map/reference issue: sequential
iteration over more than eleven ImageFrames could write beyond the state
array, and OLD left a freed old edge in the map. V9 maps by unique finite
source Headers and validates both real adjacent IMU-edge spans; missing,
extra, duplicate or mismatched keys/edges fail closed. Delayed INITIAL OLD
removes the old key inclusively and frees each owning ImageFrame edge. After
removing a reference view, it re-triangulates retained original stereo
observations using the existing triangulator; carrying depth through an
unoptimized pose transform had failed the first unchanged regression's
stereo geometry assertion. That negative source/library and log remain in
`pre_depth_reference_fix` and `native_v9_actual_init_tests_01.log`.
NON_LINEAR retains its normal optimized depth-shift behavior. Monocular
initialization retains its non-keyframe cache contract. Pointer members are
initialized explicitly so the same Estimator is valid as a local object,
and reset releases all separately owned image edges.

For this opt-in only, every already-generated native health JSON is appended
and flushed to the owned `OUTPUT_FOLDER/native_health.jsonl`, starting with
the first partial window. One extra file-only record captures the successful
seed directly before the first solve, with tracking false, prior absent and
explicit `seed_before_first_solve_and_slide` state phase; it is never published
as ROS/EV pose data. The regular same-source-stamp post-solve record proves
subsequent usable solve/prior creation. The `initialization` object snapshots source
Headers, actual frame/interval flags, waiting reason/count, exact seed means
and sample stamps/count, seed count, actual initial-prior insertion count and
valid prior state **before slide**. The current pose/IMU state fields retain
their existing post-slide phase. The normal default opens no startup log.
The logger rejects records larger than 512 KiB and exposes IO failure;
missing or incomplete trace cannot prove successful initialization. This
adds bounded synchronous file IO, which must be checked against real source
freshness/rate in ordinary diagnosticsOFF acceptance; it performs no extra
matrix evaluation or optimization.

Tests exercise the actual Estimator with original 250 Hz integration and
near stereo images: six simulated seconds of sustained image motion despite
quiet IMU must remain INITIAL, with correctly mapped OLD source windows and
no seed/prior; later complete source-confirmed static support must seed and
marginalize one prior exactly once, preserve stereo geometry and allow RP
changes in the actual first-pose chart. Subsequent actual solves cannot
reseed or insert a second initial prior. Production helper tests reject
missing/extra/duplicated source headers, wrong raw IMU edges and broken
stationary history. Existing captured-prior/factor finite-difference and
free-RP/static/motion graph tests remain required. These are offline numerical
regressions, not fresh cold-start or flight quality proof.

Limitations: source stationarity cannot distinguish every slow rotation from
sensor bias or all nearly motionless distant imagery; near stereo support
reduces but does not remove this ambiguity. The factory acceleration-bias
prior leaves genuine gravity/accelerometer uncertainty. V9 may wait forever
in unsupported or moving scenes, by design, and this simulation startup
experiment does not replace the upstream dynamic initializer for general
real-world use. A valid seed and active fusion cannot substitute for the
predeclared three fresh diagnosticsOFF cold starts, independent gravity
error, timing/rate/reset and later complete flight/post-landing gates.

### V9 visual lifecycle diagnostic (physical revision unchanged)

`PX4_AGENT_VINS_VISUAL_DIAGNOSTICS=1` explicitly makes the asset generator write
`visual_lifecycle_diagnostics: 1`. The native global and normal generated YAML
both default to zero. This is a diagnostic extension to V9, not a new tracking
or outlier policy. The original V9 source, binaries and provenance are preserved
under `logs/ev/native_v9_visual_diagnostics_design_01`.

The existing `native_health` JSON carries pre-slide, real-camera source window
headers, original IMU spans, and feature snapshots before solve, after accepted
solve and before deletion, and after actual outlier deletion. Eligibility means
only at least four backend observations. Each snapshot includes actual ID,
start-frame index, observation count, solved depth and solve flag. The existing
outlier loop supplies its actual mean of normalized reprojection norms and its
original `FOCAL_LENGTH=460` virtual-pixel conversion; no extra residual evaluation
runs. Virtual pixels are not the camera's 320-pixel focal-length image pixels.
The original strict `>3` virtual-pixel deletion test and 1.5-second visual-loss
check remain unchanged. Initial solves have no outlier stage and say so.

Each feature snapshot and reprojection list records at most 128 rows; actual
removed IDs record at most 256. Full counts, omitted counts and truncation flags
remain visible. The count does not call `getFeatureCount()` or alter `used_num`.
Nonfinite diagnostic numbers serialize as JSON null. With the flag off no extra
feature traversal, reprojection or file write occurs; health adds only the small
`enabled:false` field. With it on the existing startup JSONL writer remains
synchronous, so producer freshness must be checked in the actual run. These
snapshots can establish backend reject/rebirth behavior; raw stereo tracks alone
cannot establish optimizer eligibility or solved depth.
