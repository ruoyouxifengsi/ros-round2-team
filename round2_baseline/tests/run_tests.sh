#!/usr/bin/env bash
set -eu

tests_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
package_dir=$(dirname -- "$tests_dir")
export PYTHONPATH="$package_dir/scripts:$package_dir/tools${PYTHONPATH:+:$PYTHONPATH}"

if (( $# > 0 )); then
    test_name=$1
    shift
    exec python3 -B "$tests_dir/$test_name" "$@"
fi

for test in "$tests_dir"/test_*.py; do
    case "${test##*/}" in
        test_contact_observer.py) continue ;;
    esac
    python3 -B "$test"
done
