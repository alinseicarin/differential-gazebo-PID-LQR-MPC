# Independent confirmatory completion protocol

Status: frozen before collection of Gazebo seeds 2500--2509.

## Motivation and separation from prior data

The secondary severe campaign produced an exploratory completion contrast
under permanent left-wheel effectiveness loss. This independent campaign is a
prospective validation of that binary endpoint. No observation from the
baseline, severe, pilot, or anomaly-recheck datasets is pooled with the new
inferential sample.

## Frozen experiment matrix

- Controllers: PID, TVLQR, and LTV-MPC.
- Track: figure-eight.
- Conditions: nominal and left-wheel effectiveness reduced to 0.70 from
  experiment time 5.0 s until the observation horizon.
- Repetitions: 10 paired Gazebo seeds, 2500--2509.
- Noise seeds: 29000--29009.
- Observation horizon: 30.0 s of experiment time for every trial.
- Timed-reference configuration: `trajectory_reference_severe.yaml`.
- Controller order: deterministic rotation between repetitions.
- Total trials: 3 x 2 x 10 = 60.

The nominal condition is retained as an independent diagnostic and as the
matched baseline for continuous degradation metrics. It is not part of the
primary completion family.

## Primary endpoint

A trial is successful when the timed reference has ended and, no later than
the 30.0 s observation horizon, the controller reports both:

- estimated terminal position error no greater than 0.08 m;
- absolute estimated terminal heading error no greater than 0.15 rad.

Ground truth remains unavailable to the controller. It is logged only for the
continuous performance evaluation.

## Primary hypotheses and multiplicity

For the permanent wheel-loss condition, trajectory completion is compared for
the three paired controller contrasts:

1. TVLQR versus PID;
2. MPC versus PID;
3. MPC versus TVLQR.

Each null hypothesis states that the paired marginal completion probabilities
are equal. Each comparison uses the exact two-sided McNemar test based only on
discordant seeds. Holm correction controls the family-wise error rate at 0.05
over the three primary comparisons. The unadjusted value, Holm-adjusted value,
completion counts, rate difference, and discordant-pair counts are all
reported. No trial is removed because its result is unfavorable or extreme.

## Secondary analyses

Nominal completion is a secondary diagnostic family. Continuous accuracy,
robustness, and command-activity metrics retain the paired two-sided sign-flip
tests and engineering-relevance bands used by the frozen common analyzer.
They support interpretation but do not replace the primary completion result.

## Invalid trials and stopping rule

The campaign has no efficacy stopping rule and is analyzed only after all 60
valid trials are present. A trial may be repeated only after a documented
technical invalidation such as a missing file, failed ROS graph, or failed
protocol audit. Ordinary non-completion at 30.0 s is a valid outcome, not a
technical failure. The executor archives source, configurations, tracks,
seeds, and protocol hashes beside the results.
