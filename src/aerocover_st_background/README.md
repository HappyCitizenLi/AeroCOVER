# AeroCOVER point-level ST background

This package is AeroCOVER's standalone point-history/background stage. Each
AeroCOVER process constructs its own `BackgroundClassifier`; no labels or
state are shared between processes.

For every scan the classifier expires history outside the configured window,
builds the exact 0.30 m radius graph, forms components, checks 10 ms per-scan
slices, propagates prior background labels, returns current-point background
labels plus current residual components, and only then retains the labelled
scan as its own history. A sufficiently large first-scan slice can therefore
be background without an artificial FIFO warm-up.

The optional `connectivity_threads` path parallelizes neighboring-cell distance
checks while serializing union-find root updates. It preserves the sequential
early-connected check and produces the same point labels. The production
single-thread path carries the current cell's root across neighbor checks,
unites already-known roots directly, and refreshes that root after each merge;
the concurrent path still resolves roots under its lock. Occupied-cell lookup
uses a generation-stamped open-addressing scratch table, neighbor offsets are
visited near-first to maximize that pruning, and positions are grouped by cell
in contiguous scratch storage for the exact distance checks. Original point
indices are retained separately for unchanged output ordering. Cell-pair AABBs can
prove both definite separation and definite connection; uncertain pairs still
use the original exact point-distance loop.

A1 disables historical propagation with `use_history=false`; A2 evaluates the
whole connected history extent with `use_whole_history_extent=true`. The unit
test `IndependentConsumersProduceIdenticalPointLabels` feeds the same scans to
two independent instances and requires identical labels.
