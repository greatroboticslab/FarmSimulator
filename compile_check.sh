#!/usr/bin/env bash
# Compile the project's C# without Unity and without a Unity licence.
#
# Unity's licence on a dev machine lapses, and opening the editor costs minutes
# and gigabytes. Roslyn plus the shipped Managed DLLs catches real compile errors
# in seconds. Two passes:
#
#   runtime  - everything that goes in a player, referenced against the DLLs from
#              the last release build (Builds/ref_managed)
#   editor   - Assets/Editor, which needs UnityEditor.dll from the installed
#              editor. Those DLLs are on disk whether or not the licence is valid.
#
# Usage: bash compile_check.sh [runtime|editor|all]
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CSC="/c/Program Files/dotnet/sdk/9.0.203/Roslyn/bincore/csc.dll"
REF_RUNTIME="$REPO/Builds/ref_managed"
UNITY_MANAGED="/c/Users/gmarc/AppData/Local/Unity/Hub/Editor/2022.3.8f1/Editor/Data/Managed"
OUT="${TMPDIR:-/tmp}/farmsim_compile"
WHICH="${1:-all}"

mkdir -p "$OUT"

if [ ! -d "$REF_RUNTIME" ]; then
	echo "missing $REF_RUNTIME"
	echo "recover it with:"
	echo "  unzip -j -o '$REPO/Builds/FarmSimulator_Windows.zip' '*Managed*.dll' -d '$REF_RUNTIME'"
	exit 1
fi

# Source sets. Oculus, TextMesh Pro, XR and ml-agents are vendored third-party
# trees that do not compile standalone against these references and that we never
# touch; excluding them is what makes this run clean on a good tree.
runtime_sources() {
	find "$REPO/Assets" -name '*.cs' \
		-not -path '*/Editor/*' \
		-not -path '*/Oculus/*' \
		-not -path '*/TextMesh Pro/*' \
		-not -path '*/XR/*' \
		-not -path '*ml-agents*'
}

editor_sources() {
	find "$REPO/Assets" -name '*.cs' \
		-not -path '*/Oculus/*' \
		-not -path '*/TextMesh Pro/*' \
		-not -path '*/XR/*' \
		-not -path '*ml-agents*'
}

# Assembly-CSharp is the previous build of this same code; referencing it would
# collide with every type we are compiling from source.
# every path goes in quoted: this tree has directories like "Infinity Code" and
# "Real World Terrain", and an unquoted space in a response file silently splits
# one path into two bogus ones
# $2, when given, is a directory whose assemblies win. Any DLL in $1 with the
# same file name is skipped. The editor pass needs this: Unity's editor-time
# reference set already carries every UnityEngine module, and pulling the
# player's copies in alongside makes every Unity type ambiguous (CS0433). Doing
# it by file name rather than a name pattern keeps assemblies the editor folder
# does NOT ship, notably UnityEngine.UI, which lives in a package.
refs_from() {
	local dir="$1" dedupe="${2:-}" skip=""
	if [ -n "$dedupe" ] && [ -d "$dedupe" ]; then
		skip="$(find "$dedupe" -maxdepth 1 -name '*.dll' -exec basename {} \; | sort -u)"
	fi
	find "$dir" -maxdepth 1 -name '*.dll' | grep -v 'Assembly-CSharp' | while read -r d; do
		if [ -n "$skip" ] && printf '%s\n' "$skip" | grep -qxF "$(basename "$d")"; then
			continue
		fi
		printf -- '-r:"%s"\n' "$(cygpath -w "$d" 2>/dev/null || echo "$d")"
	done
}

run_pass() {
	local name="$1" srcfn="$2" outdll="$3"
	shift 3
	local rsp="$OUT/$name.rsp"

	{
		echo "-target:library"
		echo "-out:\"$(cygpath -w "$OUT/$outdll" 2>/dev/null || echo "$OUT/$outdll")\""
		echo "-nostdlib+"
		echo "-noconfig"
		echo "-langversion:9.0"
		# warnings are noisy in a decade-old codebase; this pass is about errors
		echo "-nowarn:0169,0414,0649,0618,3021,0067"
		# each entry is either "dir" or "dir|dir-whose-assemblies-win"
		for d in "$@"; do
			case "$d" in
				*"|"*) refs_from "${d%%|*}" "${d#*|}" ;;
				*)     refs_from "$d" ;;
			esac
		done
		"$srcfn" | while read -r f; do
			printf '"%s"\n' "$(cygpath -w "$f" 2>/dev/null || echo "$f")"
		done
	} > "$rsp"

	local count
	count=$("$srcfn" | wc -l)
	echo "=== $name pass: $count source files ==="

	local log="$OUT/$name.log"
	# a stale dll from an earlier run must not pass for this one's output
	rm -f "$OUT/$outdll"
	dotnet "$(cygpath -w "$CSC" 2>/dev/null || echo "$CSC")" \
		"@$(cygpath -w "$rsp" 2>/dev/null || echo "$rsp")" > "$log" 2>&1

	local errs
	errs=$(grep -cE 'error CS' "$log" || true)
	if [ "$errs" -eq 0 ] && [ -s "$OUT/$outdll" ]; then
		echo "$name: clean"
	elif [ "$errs" -eq 0 ]; then
		# no compiler errors but no assembly either: csc never ran properly
		tail -5 "$log"
		echo "$name: compiler produced no output (full log: $log)"
		FAILED=1
	else
		grep -E 'error CS' "$log" | sort -u | head -40
		echo "$name: $errs errors (full log: $log)"
		FAILED=1
	fi
	return 0
}

FAILED=0

if [ "$WHICH" = "runtime" ] || [ "$WHICH" = "all" ]; then
	run_pass runtime runtime_sources runtime.dll "$REF_RUNTIME"
fi

if [ "$WHICH" = "editor" ] || [ "$WHICH" = "all" ]; then
	# Unity's editor-time reference set, the same one the editor compiles editor
	# scripts against. It carries both the UnityEngine and UnityEditor modules,
	# so the player's copies must be held back to whatever is genuinely
	# third-party (M2Mqtt, Newtonsoft and friends).
	UNITY_REFS="$UNITY_MANAGED/UnityEngine"
	if [ -d "$UNITY_REFS" ]; then
		run_pass editor editor_sources editor.dll \
			"$UNITY_REFS" \
			"$REF_RUNTIME|$UNITY_REFS"
	else
		echo "editor pass skipped: no $UNITY_REFS"
	fi
fi

exit "$FAILED"
