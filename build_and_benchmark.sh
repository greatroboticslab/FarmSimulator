#!/bin/bash
# Rebuilds the farm terrain, builds the Windows player, and measures each
# graphics tier. Needs a signed-in Unity (the editor refuses to run in batch
# mode without a licence).
#
#   bash build_and_benchmark.sh              everything
#   bash build_and_benchmark.sh bench        just the benchmark, reusing the build
#   UNITY=/path/to/Unity bash build_and_benchmark.sh     other install location
#
# Results: Builds/benchmark.csv, one row per tier, plus a screenshot of each.
set -u

PROJ="$(cd "$(dirname "$0")" && pwd)"
LOGS="$PROJ/Builds/logs"
EXE="$PROJ/Builds/Windows/FarmSimulator.exe"
CSV="$PROJ/Builds/benchmark.csv"
TIERS="${TIERS:-0 1 2 3 5}"
SECONDS_PER_TIER="${SECONDS_PER_TIER:-8}"

if [ -z "${UNITY:-}" ]; then
	for guess in \
		"$LOCALAPPDATA/Unity/Hub/Editor/2022.3.8f1/Editor/Unity.exe" \
		"/c/Program Files/Unity/Hub/Editor/2022.3.8f1/Editor/Unity.exe" \
		"$HOME/Unity/Hub/Editor/2022.3.8f1/Editor/Unity"; do
		[ -x "$guess" ] && UNITY="$guess" && break
	done
fi
if [ -z "${UNITY:-}" ] || [ ! -x "$UNITY" ]; then
	echo "Unity 2022.3.8f1 not found. Set UNITY to the editor executable and run again."
	exit 1
fi

mkdir -p "$LOGS"

# Unity allows one editor per project. If one is already open on this project
# (an import in progress, or the editor itself), wait for it rather than
# crashing into its lock.
wait_for_editor() {
	local waited=0
	while [ -f "$PROJ/Temp/UnityLockfile" ] && tasklist 2>/dev/null | grep -qi "^Unity\.exe"; do
		[ "$waited" = "0" ] && echo "== another Unity instance has this project open; waiting for it to close"
		sleep 30
		waited=$((waited + 30))
		if [ "$waited" -ge 10800 ]; then
			echo "   still open after three hours; close it and run this again"
			exit 5
		fi
	done
	[ "$waited" != "0" ] && echo "   free after ${waited}s"
}

run_editor() {
	wait_for_editor
	local what="$1" method="$2" log="$LOGS/$3.log"
	echo "== $what"
	"$UNITY" -batchmode -quit -projectPath "$PROJ" -executeMethod "$method" -logFile "$log" >/dev/null 2>&1
	local code=$?
	grep -E "error CS|Exception:|BuildTools:|FarmTerrainBuilder" "$log" | head -10
	if [ "$code" != "0" ]; then
		echo "   failed (exit $code); see $log"
		if grep -q "No valid Unity Editor license" "$log"; then
			echo "   sign in to Unity Hub first"
		fi
		exit "$code"
	fi
}

if [ "${1:-all}" != "bench" ]; then
	run_editor "regenerating farm terrain" FarmTerrainBuilder.Build terrain
	run_editor "building the Windows player" BuildTools.BuildWindows build
fi

if [ ! -x "$EXE" ]; then
	echo "no player at $EXE"
	exit 1
fi

echo "== measuring tiers: $TIERS"
rm -f "$CSV"
for t in $TIERS; do
	"$EXE" -benchmark -quality "$t" -benchmark-seconds "$SECONDS_PER_TIER" \
		-benchmark-out "$CSV" -screen-fullscreen 0 -logFile "$LOGS/bench_q$t.log"
	grep -h "\[Benchmark\]" "$LOGS/bench_q$t.log" | tail -1
done

echo
echo "== $CSV"
cat "$CSV"
