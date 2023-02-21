#!/bin/bash -axe

test -n "$plot_title"
test -n "$plot_legend"
test -n "$plot_xlabel"
test -n "$plot_ylabel"
test -n "$plot_values"

plot_data()
{
    title="$1"
    shift
    legend="$1"
    shift
    xlabel="$1"
    shift
    ylabel="$1"
    shift
    values="$@"

    data_file="$(mktemp)"
    echo "$values" > "$data_file"

    plot_file="$(mktemp)"
    random_hex_color="$(cat /dev/urandom | xxd -p -l 3)"
    test -z "$plot_to_filename" && echo "set terminal x11"                                                    >> "$plot_file"
    test -z "$plot_to_filename" && echo "set output"                                                          >> "$plot_file"
    test -n "$plot_to_filename" && echo "set terminal png"                                                    >> "$plot_file"
    test -n "$plot_to_filename" && echo "set output \"$plot_to_filename\""                                    >> "$plot_file"
    echo "set title \"$title\" noenhanced"                                                                    >> "$plot_file"
    echo "set xlabel \"$xlabel\" noenhanced"                                                                  >> "$plot_file"
    echo "set ylabel \"$ylabel\" noenhanced"                                                                  >> "$plot_file"
    echo "plot '$data_file' w linespoints lw 2 lt rgb \"#${random_hex_color}\" title \"$legend\" noenhanced " >> "$plot_file"
    test -z "$plot_to_filename" && echo "pause mouse"                                                         >> "$plot_file"

    gnuplot "$plot_file"
    rm "$plot_file"
    rm "$data_file"
}

plot_data "$plot_title" "$plot_legend" "$plot_xlabel" "$plot_ylabel" "$plot_values"
