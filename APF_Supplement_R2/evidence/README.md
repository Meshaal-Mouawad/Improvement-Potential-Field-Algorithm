# APF experiment evidence

There are 3,600 development, 3,552 held-out and 192 post-hoc episodes, totaling 7,344 scalar records. core.js and reproduce.js preserve the corrected JavaScript experiment; original_2021 preserves the MATLAB source snapshot.

## Clean extraction workflow

Requires Node.js and Python 3 without third-party dependencies. Run from this directory:

    node reproduce.js > /tmp/apf_reproduction.json
    node analyze.cjs > /tmp/apf_statistics.json
    python3 verify.py --statistics /tmp/apf_statistics.json --trajectories /tmp/apf_reproduction.json
    node negative_controls.cjs

Regenerate the complete original statistical format:

    node analyze.cjs --legacy > /tmp/apf_legacy_statistics.json
    python3 -c "import json; assert json.load(open('/tmp/apf_legacy_statistics.json')) == json.load(open('paired_statistics.json'))"

Generation, analysis and validation are separate programs. reproduce.js emits all maps, selections, metrics and trajectories. analyze.cjs reads metrics.json and prints statistics. verify.py delegates to verify.cjs, which reads files and reports failures with nonzero exit status. Validation never rewrites data, provenance, statistics or manifests. Explicit shell redirection chooses temporary generation destinations.

## Statistical protocol

statistics_r1.json contains all 16 original condition-versus-reference contrasts and both direct u16 recovery contrasts, with full six-status transitions. The --legacy analysis option reproduces the complete paired_statistics.json structure with exact JSON-value equality.

Each contrast pairs 120 maps. Ten thousand shared bootstrap samples draw 40 maps with replacement within each of single, gate and clutter. LCG32 seed 20261106 updates state = (1664525 * state + 1013904223) modulo 2^32 and returns u = state / 2^32. Iteration order is replicate, family single/gate/clutter, then 40 draws. Numeric map suffixes 0 through 39 define each family order; the drawn index is floor(40u). All 18 contrasts share identical resampling arrays. The 95% percentile interval uses nearest-rank indices ceil(p * 10000) - 1 for p=.025 and .975, giving zero-based indices 249 and 9749. Differences are comparison minus baseline; multiply by 100 for percentage points. Continuous differences average per-map differences among maps successful under both arms.

## Validation

The validator checks 34 SHA-256 file digests; 7,344 unique keys; exact baseline scalar digest; split, family, tag and status aggregates; JSON, JSONL and CSV agreement; numeric and outcome invariants; every new statistic; and all original paired, summary, family and post-hoc statistics. --statistics additionally checks independently generated statistics.

--trajectories checks regenerated maps, rejection draws, selections and all 7,344 trajectories. Independent closest-point geometry checks every accepted segment, path lengths, workspace bounds, goal arrival, timeout budgets and minimum clearance. Collision minimum clearance includes the rejected attempted segment in the archived records. Discrete outcomes, steps and path lengths agree exactly; observed clearance roundoff is at most 2.43e-13. Continuous replay tolerance is 1e-9; archived scalar integrity remains exact.

negative_controls.cjs rejects eleven in-memory mutations and tests two temporary package mutations through the actual CLI. Updated temporary integrity manifests force coordinated outcome and statistic changes to fail semantic checks with exit status 1. The test confirms evidence files remain unchanged and removes temporary copies.

The original FNV-1a32 core identifier is d0c7248f (UTF-16 code units). SHA-256 integrity is recorded in integrity_r1.json. Map identifiers repeat across splits; use split/tag with map.

