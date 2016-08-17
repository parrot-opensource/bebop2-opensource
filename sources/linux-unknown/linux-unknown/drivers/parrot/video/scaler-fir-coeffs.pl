#! /usr/bin/env perl
#
# Script used to generate the FIR coeffs for the AVI scaler using lanczos.
#
# Author: Lionel Flandrin <lionel.flandrin@parrot.com>
# Date: 19-Jun-2013

use strict;
use warnings;

use Math::Trig;

sub gcd
{
    my ($a, $b) = @_;

    while ($a > 0) {
        my $r = $b % $a;
        $b = $a;
        $a = $r;
    }

    return $b;
}

sub make_coprime
{
    my ($a, $b) = @_;

    my $d = gcd($$a, $$b);

    $$a /= $d;
    $$b /= $d;
}

sub round_power2
{
    # Assumes $v is a 32bit integer
    my ($v) = @_;

    $v--;
    $v |= $v >> $_ foreach (1,2,4,8,16);

    return $v + 1;
}

sub sinc
{
    my ($x) = @_;

    return 1
        if ($x == 0);

    my $xpi = pi * $x;

    return sin($xpi) / $xpi;
}

sub lanczos
{
    # $n is the number of lobes (3 for lanczos 3).
    my ($n, $x) = @_;

    $x = abs($x);

    # The function is 0 when we're outside of the lanczos window
    return 0
        if ($x > $n);

    return sinc($x) * sinc($x / $n);
}

if (@ARGV < 2) {
    print "usage: $0 insize outsize [nlobes [blur]]\n";
    exit 1;
}

my ($in, $out, $nlobe, $blur) = @ARGV;

# Default to 2 lobes (lanczos2)
$nlobe = 2 if (!$nlobe);
$blur = 1.0 if (!$blur);

make_coprime(\$in, \$out);

# Reduction factor is 1.0 in case of upscale
my $reduction_factor = ($out < $in) ? (($in * 1.0) / $out) : 1.0;

$reduction_factor *= $blur;

my $ntap = int(2 * $nlobe * $reduction_factor + 0.5);

# Hardware can't handle more than 16 taps
if ($ntap > 16) {
    print "#warning Dowscale factor $reduction_factor is too large!\n";
    print "#warning We would need $ntap taps for good results\n";
    $ntap = 16;
}

# ntap has to be a power of two
$ntap = round_power2($ntap);

my $nphase = 32 / $ntap;

my $tapw = $ntap / 2;
my $pincr = 1.0 / $nphase;

my @tapcoeffs;

for my $tap ((-$tapw + 1)..$tapw) {
    my @coeffs;
    for my $phase (0..($nphase - 1)) {
        my $x = $tap - $pincr * $phase;
        # We need to translate $x in the initial image coordinates
        $x /= $reduction_factor;
        push(@coeffs, lanczos($nlobe, $x));
    }
    push @tapcoeffs, \@coeffs;
}

# We now normalize each phase to sum to 64
for my $phase (0..($nphase - 1)) {
    my $sum = 0.;
    for my $tap (0 .. ($ntap - 1)) {
        $sum += $tapcoeffs[$tap][$phase];
    }
    my $factor = 64 / $sum;

    $sum = 0;

    for my $tap (0 .. ($ntap - 1)) {
        my $coeff = \$tapcoeffs[$tap][$phase];

        $$coeff *= $factor;
        $$coeff = int($$coeff + 0.5);
        $sum += $$coeff;
    }

    # If sum is too big, we lower the outer parts of the FIR
    for (my $i = 0; ($i < $ntap) && ($sum > 64); $i++) {
        my $tap = $i >> 1;
        $tap = $ntap - $tap - 1
            if ($i & 1);
        $tapcoeffs[$tap][$phase]--;
        $sum--;
    }

    for (my $i = 0; ($i < $ntap) && ($sum < 64); $i++) {
        my $center = ($ntap - 1) / 2;
        my $tap = $i >> 1;
        $tap = -$tap - 1
            if ($i & 1);
        $tapcoeffs[$center - $tap][$phase]--;
        $sum--;
    }
}

sub set_point
{
    my ($lines, $x, $y, $c) = @_;

    substr($lines->[$y], $x, 1) = $c;
}

sub graph_taps
{
    my ($coeffs, $height) = @_;

    $height = 64
        if (!$height);

    my @tapcoeffs = @$coeffs;

    my @lines;

    push @lines, ' ' x 32
        foreach (0 .. ($height - 1));

    # y-axis
    set_point(\@lines, 15, $_, '|')
        foreach (0 .. ($height - 1));

    # x-axis
    set_point(\@lines, $_, $height / 2, '-')
        foreach (0 .. 31);

    my $min_y = $height;
    my $max_y = 0;

    for my $tap (0 .. ($ntap - 1)) {
        for my $p (1..$nphase) {
            my $phase = $nphase - $p;
            my $x = $tap * $nphase + ($p - 1);
            my $y = $tapcoeffs[$tap][$phase] + 64;

            $y = int(($y * ($height - 1)) / (2 * 64) + 0.5);

            $min_y = $y
                if ($min_y > $y);
            $max_y = $y
                if ($max_y < $y);

            set_point(\@lines, $x, $y, '+')
        }
    }

    foreach (1 .. $height) {
        my $y = $height - $_;

        next
            if ($y < $min_y || $y > $max_y);

        $lines[$y] =~ s/\s*$//;
        print ' * ', $lines[$y], "\n";
    }
}

print <<EOF;
/* Configuration for scaling factor $out / $in using lanczos
 * with $nlobe lobes.
 *
 * This configuration uses $ntap taps and $nphase phases.
 *
EOF

graph_taps(\@tapcoeffs, 33);

print <<EOF;
 */
static const struct avi_scal_dimcfg avi_scalcfg_${out}_$in = {
\t.ntaps   = AVI_SCAL_NTAPS_$ntap,
\t.nphases = AVI_SCAL_NPHASES_$nphase,
\t.coeffs = {
EOF

for my $tap (0 .. ($ntap - 1)) {
    print("\t\t",
          join(', ', map { sprintf("%3d", $_) } @{$tapcoeffs[$tap]}),
          ", /* tap $tap */\n");
}

print <<EOF;
\t},
};
EOF
