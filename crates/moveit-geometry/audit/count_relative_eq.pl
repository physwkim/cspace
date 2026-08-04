#!/usr/bin/env perl
# Usage: count_relative_eq.pl <file.rs> [<file.rs> ...]
#
# Round 19 item 1: run against `crates/moveit-geometry/audit/*` itself and
# against a synthetic block-comment/string-literal fixture -- both found this
# script counted its own doc-comment example as two live calls (its own `#`
# Perl-comment lines are not `//`-stripped) and counted a fake call written
# inside a `/* */` block comment or a `"..."` string literal as real. Neither
# false positive changed round 18's committed geometry/octomap counts (grep
# confirmed no `/* */` block comment or brace-bearing string literal exists
# in the `.rs` files those counts were taken from), but both are closed here
# so a sibling panel copying this script does not inherit them.
#
# Strips `/* */` block comments (including multi-line) and `//` line-comment
# tails, then blanks the contents of every `"..."` string literal (handles
# `\"` escapes; this repo's real call sites use no raw (`r"..."`) or byte
# (`b"..."`) string literals, confirmed by `rg`, so those forms are not
# handled), then finds every remaining `assert_relative_eq!(` /
# `relative_eq!(` call by bracket-matching parens from the macro name to the
# matching close, and classifies each by whether `epsilon =` and/or
# `max_relative =` appear inside that call's own argument text.
use strict;
use warnings;
my ($both, $eps_only, $mr_only, $neither) = (0,0,0,0);
for my $file (@ARGV) {
    open(my $fh, '<', $file) or die "$file: $!";
    local $/;
    my $text = <$fh>;
    close $fh;
    $text =~ s{/\*.*?\*/}{}gs;
    $text =~ s{//[^\n]*}{}g;
    $text =~ s{"(?:[^"\\]|\\.)*"}{""}g;
    while ($text =~ /\b(?:assert_relative_eq|relative_eq)!\s*\(/g) {
        my $open = rindex($text, '(', pos($text) - 1);
        my $depth = 0;
        my $j = $open;
        my $len = length($text);
        for (; $j < $len; $j++) {
            my $c = substr($text, $j, 1);
            $depth++ if $c eq '(';
            $depth-- if $c eq ')';
            last if $depth == 0;
        }
        my $call = substr($text, $open, $j - $open + 1);
        my $has_eps = $call =~ /\bepsilon\s*=/;
        my $has_mr  = $call =~ /\bmax_relative\s*=/;
        if ($has_eps && $has_mr) { $both++ }
        elsif ($has_eps) { $eps_only++ }
        elsif ($has_mr) { $mr_only++ }
        else { $neither++ }
        printf STDERR "%s: call at byte %d -> eps=%d mr=%d\n", $file, $open, $has_eps, $has_mr;
    }
}
print "both=$both epsilon_only=$eps_only max_relative_only=$mr_only neither=$neither\n";
