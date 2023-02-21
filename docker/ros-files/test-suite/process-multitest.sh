#!/bin/bash -ax

set -e
multitest_dir="$1"
reference_image="$2"
test -n "$multitest_dir"
test -n "$reference_image"
set +e

cleanup()
{
    echo "process-multitest: cleaning up"
    pkill -P $$ -SIGTERM
    exit $1
}

trap "cleanup 0" SIGINT SIGTERM
trap "cleanup -1" SIGHUP

leaf_dirs="$(find "$multitest_dir" -type d -links 2)"

for dir in $leaf_dirs
do
    echo "process-multitest: processing $dir"
    pgm_files="$(find "$dir" -name '*.pgm')"
    param_name="$(basename "$dir")"

    for pgm in $pgm_files
    do
        echo "process-multitest: processing file $pgm"
        score="$(python3 compare.py -a "$reference_image" -b "$pgm" -o "${pgm}.score.png" -j 3 -k 0.02 -l 0.9)"
        echo "$score" > "${pgm}.score"
    done

    echo "process-multitest: generating graph for $dir"
    param_values="$(ls -1 "$dir"/*.pgm | xargs -n1 basename | sort | sed 's/\.pgm$//')"
    scores="$(ls -1 "$dir"/*.pgm.score | sort | xargs -n 1 cat)"
    plot_values="$( paste -d "\t" <(echo "$param_values") <(echo "$scores") )"
    plot_title="$param_name vs score" \
    plot_legend="$param_name" \
    plot_xlabel="$param_name" \
    plot_ylabel="score" \
    plot_to_filename="$dir/score.png" \
    ./plot.sh
done
