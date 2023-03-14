#!/bin/bash -ae

test -n "$plot_legend"
test -n "$plot_xlabel"
test -n "$plot_ylabel"
test -n "$plot_values"

plot_data()
{
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
    test -z "$plot_to_filename" && echo "set terminal x11"                                         >> "$plot_file"
    test -z "$plot_to_filename" && echo "set output"                                               >> "$plot_file"
    test -n "$plot_to_filename" && echo "set terminal png"                                         >> "$plot_file"
    test -n "$plot_to_filename" && echo "set output \"$plot_to_filename\""                         >> "$plot_file"
    echo "set xlabel \"$xlabel\" noenhanced"                                                       >> "$plot_file"
    echo "set ylabel \"$ylabel\" noenhanced"                                                       >> "$plot_file"
    echo "stats '$data_file'"                                                                      >> "$plot_file"
    echo "set yrange [STATS_min_y-0.05:STATS_max_y+0.05]"                                          >> "$plot_file"
    echo "set key box opaque outside horizontal top center"                                        >> "$plot_file"
    echo "set border back"                                                                         >> "$plot_file"
    echo "f(x) = m*x + b"                                                                          >> "$plot_file"
    echo "fit f(x) '$data_file' via m,b"                                                           >> "$plot_file"
    echo "plot \
        '$data_file' \
        w points \
        lt rgb \"#${random_hex_color}\" \
        lw 2 \
        title \"$legend\" noenhanced , \
        f(x) \
        lt rgb \"#000000\" \
        lw 2 \
        title 'linear regression'"                                                                 >> "$plot_file"
    test -z "$plot_to_filename" && echo "pause mouse"                                              >> "$plot_file"

    gnuplot "$plot_file"
    rm -f "$plot_file"
    rm -f "$data_file"
}

plot_data "$plot_legend" "$plot_xlabel" "$plot_ylabel" "$plot_values"
